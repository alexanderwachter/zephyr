/*
 * Copyright (c) 2019 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/can.h>
#include <kernel.h>
#include <sys/util.h>
#include "can_common.h"

#define LOG_LEVEL CONFIG_CAN_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(can_driver);

#define CAN_SYNC_SEG 1

#define WORK_BUF_COUNT_IS_POWER_OF_2 !(CONFIG_CAN_WORKQ_FRAMES_BUF_CNT & \
					(CONFIG_CAN_WORKQ_FRAMES_BUF_CNT - 1))

#define WORK_BUF_MOD_MASK (CONFIG_CAN_WORKQ_FRAMES_BUF_CNT - 1)

#if WORK_BUF_COUNT_IS_POWER_OF_2
#define WORK_BUF_MOD_SIZE(x) ((x) & WORK_BUF_MOD_MASK)
#else
#define WORK_BUF_MOD_SIZE(x) ((x) % CONFIG_CAN_WORKQ_FRAMES_BUF_CNT)
#endif

#define WORK_BUF_FULL 0xFFFF

static void can_msgq_put(struct zcan_frame *frame, void *arg)
{
	struct k_msgq *msgq = (struct k_msgq *)arg;
	int ret;

	__ASSERT_NO_MSG(msgq);

	ret = k_msgq_put(msgq, frame, K_NO_WAIT);
	if (ret) {
		LOG_ERR("Msgq %p overflowed. Frame ID: 0x%x", arg, frame->id);
	}
}

int z_impl_can_attach_msgq(const struct device *dev, struct k_msgq *msg_q,
			   const struct zcan_filter *filter)
{
	const struct can_driver_api *api = dev->api;

	return api->attach_isr(dev, can_msgq_put, msg_q, filter);
}

static inline void can_work_buffer_init(struct can_frame_buffer *buffer)
{
	buffer->head = 0;
	buffer->tail = 0;
}

static inline int can_work_buffer_put(struct zcan_frame *frame,
				      struct can_frame_buffer *buffer)
{
	uint16_t next_head = WORK_BUF_MOD_SIZE(buffer->head + 1);

	if (buffer->head == WORK_BUF_FULL) {
		return -1;
	}

	buffer->buf[buffer->head] = *frame;

	/* Buffer is almost full */
	if (next_head == buffer->tail) {
		buffer->head = WORK_BUF_FULL;
	} else {
		buffer->head = next_head;
	}

	return 0;
}

static inline
struct zcan_frame *can_work_buffer_get_next(struct can_frame_buffer *buffer)
{
	/* Buffer empty */
	if (buffer->head == buffer->tail) {
		return NULL;
	} else {
		return &buffer->buf[buffer->tail];
	}
}

static inline void can_work_buffer_free_next(struct can_frame_buffer *buffer)
{
	uint16_t next_tail = WORK_BUF_MOD_SIZE(buffer->tail + 1);

	if (buffer->head == buffer->tail) {
		return;
	}

	if (buffer->head == WORK_BUF_FULL) {
		buffer->head = buffer->tail;
	}

	buffer->tail = next_tail;
}

static void can_work_handler(struct k_work *work)
{
	struct zcan_work *can_work = CONTAINER_OF(work, struct zcan_work,
						  work_item);
	struct zcan_frame *frame;

	while ((frame = can_work_buffer_get_next(&can_work->buf))) {
		can_work->cb(frame, can_work->cb_arg);
		can_work_buffer_free_next(&can_work->buf);
	}
}

static void can_work_isr_put(struct zcan_frame *frame, void *arg)
{
	struct zcan_work *work = (struct zcan_work *)arg;
	int ret;

	ret = can_work_buffer_put(frame, &work->buf);
	if (ret) {
		LOG_ERR("Workq buffer overflow. Msg ID: 0x%x", frame->id);
		return;
	}

	k_work_submit_to_queue(work->work_queue, &work->work_item);
}

int can_attach_workq(const struct device *dev, struct k_work_q *work_q,
			    struct zcan_work *work,
			    can_rx_callback_t callback, void *callback_arg,
			    const struct zcan_filter *filter)
{
	const struct can_driver_api *api = dev->api;

	k_work_init(&work->work_item, can_work_handler);
	work->work_queue = work_q;
	work->cb = callback;
	work->cb_arg = callback_arg;
	can_work_buffer_init(&work->buf);

	return api->attach_isr(dev, can_work_isr_put, work, filter);
}

bool can_check_timeout(struct can_send_ctx *ctx)
{
	return ctx->timeout.ticks >= k_uptime_ticks();
}

static struct can_send_ctx *can_get_next_tx_to(struct can_tx_driver_ctx *ctx)
{
	sys_snode_t *node = sys_slist_peek_head(&ctx->send_list);
	struct can_send_ctx *send_ctx;
	struct can_send_ctx *next_to = CONTAINER_OF(node, struct can_send_ctx, node);
	node = sys_slist_peek_next(node);

	if (!node) {
		return next_to;
	}

	SYS_SLIST_ITERATE_FROM_NODE(&ctx->send_list, node) {
		send_ctx = CONTAINER_OF(node, struct can_send_ctx, node);
		if (next_to->timeout.ticks < send_ctx->timeout.ticks) {
			next_to = send_ctx;
		}
	}

	return next_to;
}

static void can_tx_timeout_handle(struct can_tx_driver_ctx *ctx,
				  struct can_send_ctx *send_ctx)
{
	sys_slist_find_and_remove(&ctx->send_list, &send_ctx->node);
	send_ctx->cb(NULL /*TODO get dev*/, send_ctx->user_data, CAN_TX_TIMEOUT);
}

static void can_tx_timeout(struct _timeout *to)
{
	struct can_tx_driver_ctx *ctx =
		CONTAINER_OF(to, struct can_tx_driver_ctx, to);
	struct can_send_ctx *next_to;
	struct  z_spinlock_key key;
	k_timeout_t next_timeout;

	can_tx_timeout_handle(ctx, ctx->next_to);

	key = k_spin_lock(&ctx->lock);

	while (!sys_slist_is_empty(&ctx->send_list)) {
		next_to = can_get_next_tx_to(ctx);
		k_spin_unlock(&ctx->lock, key);
		if (next_to->timeout.ticks <= k_uptime_ticks()) {
			can_tx_timeout_handle(ctx, next_to);
			key = k_spin_lock(&ctx->lock);
			continue;
		}

		next_timeout.ticks = next_to->timeout.ticks - k_uptime_ticks();
		z_add_timeout(&ctx->to, can_tx_timeout, next_timeout);
		break;
	}
}

static int can_abort_tx(struct can_tx_driver_ctx *ctx,
			struct can_send_ctx *send_ctx)
{
	k_timeout_t rem_ticks;
	struct z_spinlock_key key;

	key = k_spin_lock(&ctx->lock);

	if (send_ctx == ctx->next_to) {
		ctx->next_to = can_get_next_tx_to(ctx);
		rem_ticks.ticks =  ctx->next_to->timeout.ticks - k_uptime_ticks();
		z_abort_timeout(&ctx->to);
		z_add_timeout(&ctx->to, can_tx_timeout, rem_ticks);
	}

	sys_slist_find_and_remove(&ctx->send_list, &send_ctx->node);

	k_spin_unlock(&ctx->lock, key);

	send_ctx->cb(NULL /*dev*/, send_ctx->user_data, CAN_TX_ABORT);

	return CAN_TX_OK;
}

bool can_frame_prio_higher(const struct zcan_frame *frame1,
			   const struct zcan_frame *frame2)
{
	if (frame1->id_type == frame2->id_type) {
		return frame1->id > frame2->id;
	}

	/* standard ID has higher prio than extended ID */
	if (frame1->id_type == CAN_STANDARD_IDENTIFIER) {
		return true;
	}

	return false;
}

bool can_frame_prio_higher_equal(const struct zcan_frame *frame1,
				 const struct zcan_frame *frame2)
{
	if (frame1->id_type == frame2->id_type) {
		return frame1->id >= frame2->id;
	}

	/* standard ID has higher prio than extended ID */
	if (frame1->id_type == CAN_STANDARD_IDENTIFIER) {
		return true;
	}

	return false;
}

static inline void can_insert_tx_ctx(struct can_tx_driver_ctx *ctx,
				     struct can_send_ctx *send_ctx)
{
	sys_snode_t *prev_node = sys_slist_peek_head(&ctx->send_list);
	sys_snode_t *node = sys_slist_peek_next(prev_node);
	struct can_send_ctx *send_ctx_next;
	const struct zcan_frame *frame = send_ctx->frame;

	if (prev_node == NULL ||
	    can_frame_prio_higher(frame,
				  CONTAINER_OF(prev_node, struct can_send_ctx, node)->frame)) {
		sys_slist_append(&ctx->send_list, &send_ctx->node);
	}

	if (node == NULL) {
		sys_slist_prepend(&ctx->send_list, &send_ctx->node);
		return;
	}

	SYS_SLIST_ITERATE_FROM_NODE(&ctx->send_list, node) {
		send_ctx_next = CONTAINER_OF(node, struct can_send_ctx, node);
		if (can_frame_prio_higher(frame, send_ctx_next->frame)) {
			sys_slist_insert(&ctx->send_list, prev_node,
					 &send_ctx->node);
			return;
		}
	}

	sys_slist_prepend(&ctx->send_list, &send_ctx->node);
}

void can_put_back_tx(struct can_tx_driver_ctx *ctx,
		     struct can_send_ctx *send_ctx)
{
	sys_snode_t *node = sys_slist_peek_tail(&ctx->send_list);
	const struct zcan_frame *frame = send_ctx->frame;
	struct can_send_ctx *send_ctx_next;

	if (node == 0) {
		sys_slist_append(&ctx->send_list, &send_ctx->node);
		return;
	}

	send_ctx_next = CONTAINER_OF(node, struct can_send_ctx, node);
	if (can_frame_prio_higher_equal(frame, send_ctx_next->frame)) {
		sys_slist_prepend(&ctx->send_list, &send_ctx->node);
		return;
	}

	SYS_SLIST_ITERATE_FROM_NODE(&ctx->send_list, node) {
		send_ctx_next = CONTAINER_OF(node, struct can_send_ctx, node);
		if (!can_frame_prio_higher_equal(frame, send_ctx_next->frame)) {
			sys_slist_insert(&ctx->send_list, node, &send_ctx->node);
		}
	}
}

int can_queue_tx(struct can_tx_driver_ctx *ctx, struct can_send_ctx *send_ctx)
{
	k_timeout_t rem_ticks;
	struct z_spinlock_key key;

	if (k_uptime_ticks() >= send_ctx->timeout.ticks) {
		return CAN_TX_TIMEOUT;
	}

	rem_ticks.ticks = send_ctx->timeout.ticks - k_uptime_ticks();

	key = k_spin_lock(&ctx->lock);
	can_insert_tx_ctx(ctx, send_ctx);

	if (rem_ticks.ticks < z_timeout_remaining(&ctx->to)) {
		z_abort_timeout(&ctx->to);
		z_add_timeout(&ctx->to, can_tx_timeout, rem_ticks);
		ctx->next_to = send_ctx;
	}

	k_spin_unlock(&ctx->lock, key);

	return CAN_TX_OK;
}

int can_send_async(const struct device *dev, k_timeout_t frame_timeout,
			  struct can_send_ctx *send_ctx)
{
	const struct can_driver_api *api =
		(const struct can_driver_api *)dev->api;
	struct can_tx_driver_ctx *ctx = (struct can_tx_driver_ctx *) dev->data;
	int ret;

	if (!send_ctx->cb || !send_ctx->frame || !send_ctx->frames_cnt) {
		LOG_ERR("Invalid send_ctx");
		return CAN_TX_EINVAL;
	}

	send_ctx->timeout.ticks = frame_timeout.ticks + k_uptime_ticks();

	ret = api->send(dev, send_ctx);
	if (ret == CAN_TX_BUSY) {
		ret = can_queue_tx(ctx, send_ctx);
	}

	return ret;
}

struct can_cb_data {
	struct k_sem sem;
	int res;
};

static void can_send_cb(const struct device *dev, void * user_data, int res)
{
	struct can_cb_data *data = (struct can_cb_data *)user_data;
	data->res = res;
	k_sem_give(&data->sem);
}

int z_impl_can_send(const struct device *dev, const struct zcan_frame *frame,
		    k_timeout_t frame_timeout)
{
	struct can_cb_data data;
	struct can_send_ctx ctx;
	int res;

	k_sem_init(&data.sem, 0, 1);
	can_send_ctx_init(&ctx, frame, 1, can_send_cb, &data);
	res = can_send_async(dev, frame_timeout, &ctx);
	if (res != CAN_TX_OK) {
		return res;
	}

	k_sem_take(&data.sem, K_FOREVER);
	return data.res;
}

static int update_sampling_pnt(uint32_t ts, uint32_t sp, struct can_timing *res,
			       const struct can_timing *max,
			       const struct can_timing *min)
{
	uint16_t ts1_max = max->phase_seg1 + max->prop_seg;
	uint16_t ts1_min = min->phase_seg1 + min->prop_seg;
	uint32_t sp_calc;
	uint16_t ts1, ts2;

	ts2 = ts - (ts * sp) / 1000;
	ts2 = CLAMP(ts2, min->phase_seg2, max->phase_seg2);
	ts1 = ts - CAN_SYNC_SEG - ts2;

	if (ts1 > ts1_max) {
		ts1 = ts1_max;
		ts2 = ts - CAN_SYNC_SEG - ts1;
		if (ts2 > max->phase_seg2) {
			return -1;
		}
	} else if (ts1 < ts1_min) {
		ts1 = ts1_min;
		ts2 = ts - ts1;
		if (ts2 < min->phase_seg2) {
			return -1;
		}
	}

	res->prop_seg = CLAMP(ts1 / 2, min->prop_seg, max->prop_seg);
	res->phase_seg1 = ts1 - res->prop_seg;
	res->phase_seg2 = ts2;

	sp_calc = (CAN_SYNC_SEG + ts1 * 1000) / ts;

	return sp_calc > sp ? sp_calc - sp : sp - sp_calc;
}

/* Internal function to do the actual calculation */
static int can_calc_timing_int(uint32_t core_clock, struct can_timing *res,
			       const struct can_timing *min,
			       const struct can_timing *max,
			       uint32_t bitrate, uint16_t sp)
{
	uint32_t ts = max->prop_seg + max->phase_seg1 + max->phase_seg2 +
		   CAN_SYNC_SEG;
	uint16_t sp_err_min = UINT16_MAX;
	int sp_err;
	struct can_timing tmp_res;

	if (sp >= 1000 ||
	    (!IS_ENABLED(CONFIG_CAN_FD_MODE) && bitrate > 1000000) ||
	     (IS_ENABLED(CONFIG_CAN_FD_MODE) && bitrate > 8000000)) {
		return -EINVAL;
	}

	for (int prescaler = MAX(core_clock / (ts * bitrate), 1);
	     prescaler <= max->prescaler; ++prescaler) {
		if (core_clock % (prescaler * bitrate)) {
			/* No integer ts */
			continue;
		}

		ts = core_clock / (prescaler * bitrate);

		sp_err = update_sampling_pnt(ts, sp, &tmp_res,
					     max, min);
		if (sp_err < 0) {
			/* No prop_seg, seg1, seg2 combination possible */
			continue;
		}

		if (sp_err < sp_err_min) {
			sp_err_min = sp_err;
			*res = tmp_res;
			res->prescaler = (uint16_t)prescaler;
			if (sp_err == 0) {
				/* No better result than a perfect match*/
				break;
			}
		}
	}

	if (sp_err_min) {
		LOG_DBG("SP error: %d 1/1000", sp_err_min);
	}

	return sp_err_min == UINT16_MAX ? -EINVAL : (int)sp_err_min;
}

int can_calc_timing(const struct device *dev, struct can_timing *res,
		    uint32_t bitrate, uint16_t sample_pnt)
{
	const uint32_t core_clock = can_get_core_clock(dev);
	const struct can_driver_api *api = dev->api;

	return can_calc_timing_int(core_clock, res, &api->timing_min,
				   &api->timing_max, bitrate, sample_pnt);
}

#ifdef CONFIG_CAN_FD_MODE
int can_calc_timing_data(const struct device *dev, struct can_timing *res,
			 uint32_t bitrate, uint16_t sample_pnt)
{
	const uint32_t core_clock = can_get_core_clock(dev);
	const struct can_driver_api *api = dev->api;

	return can_calc_timing_int(core_clock, res, &api->timing_min_data,
				   &api->timing_max_data, bitrate, sample_pnt);
}
#endif

int can_calc_prescaler(const struct device *dev, struct can_timing *timing,
		       uint32_t bitrate)
{
	const uint32_t core_clock = can_get_core_clock(dev);
	uint32_t ts = timing->prop_seg + timing->phase_seg1 + timing->phase_seg2 +
		   CAN_SYNC_SEG;

	timing->prescaler = core_clock / (bitrate * ts);

	return core_clock % (ts * timing->prescaler);
}
