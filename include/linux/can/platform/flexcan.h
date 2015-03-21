/*
 * Copyright (C) 2010 Marc Kleine-Budde <kernel@pengutronix.de>
 *
 * This file is released under the GPLv2
 *
 */

#ifndef __CAN_PLATFORM_FLEXCAN_H
#define __CAN_PLATFORM_FLEXCAN_H

/**
 * struct flexcan_platform_data - flex CAN controller platform data
 * @transceiver_switch:         - called to power on/off the transceiver
 * @gpio_switch:         		- called to power on/off the gpio indicator
 *
 */
struct flexcan_platform_data {
	void (*transceiver_switch)(int enable);
	void (*gpio_switch)(int enable);
};

#endif /* __CAN_PLATFORM_FLEXCAN_H */
