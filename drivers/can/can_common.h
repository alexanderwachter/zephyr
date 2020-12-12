/*
 * Copyright (c) 2020 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_DRIVERS_CAN_COMMON_H_
#define ZEPHYR_DRIVERS_CAN_COMMON_H_

#include <drivers/can.h>
#include <timeout_q.h>

/* This context has to be at the beginning of each device data struct */
struct can_tx_driver_ctx {
	sys_slist_t send_list;
	struct _timeout to;
	struct can_send_ctx *next_to;
	struct k_spinlock lock;
};

bool can_frame_prio_higher(const struct zcan_frame *frame1,
			   const struct zcan_frame *frame2);
void can_put_back_tx(struct can_tx_driver_ctx *ctx,
		     struct can_send_ctx *send_ctx);
bool can_check_timeout(struct can_send_ctx *ctx);

#endif /*ZEPHYR_DRIVERS_CAN_COMMON_H_*/
