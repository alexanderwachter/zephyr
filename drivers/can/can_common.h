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

struct can_mailbox_ctx {
	struct can_send_ctx *send_ctx;
};

/* This context has to be at the beginning of each device data struct */
struct can_tx_driver_ctx {
	sys_slist_t send_list;
	struct _timeout to;
	struct can_send_ctx *next_to;
	struct k_spinlock lock;
	struct can_mailbox_ctx *mailboxes;
	size_t mailboxes_len;
};

void can_common_mailbox_empty(struct device* dev, struct can_tx_driver_ctx *ctx, size_t mailbox_nr, int reason);

int can_common_send_async(struct can_tx_driver_ctx *ctx, k_timeout_t frame_timeout,
			  struct can_send_ctx *send_ctx);

#endif /*ZEPHYR_DRIVERS_CAN_COMMON_H_*/
