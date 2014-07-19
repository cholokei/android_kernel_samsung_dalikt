/*
 * Copyright (C) 2010 Samsung Electronics
 * Minkyu Kang <mk7.kang@samsung.com>
 * Wonguk Jeong <wonguk.jeong@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _FSA9480_H_
#define _FSA9480_H_

#define FSA9480_ATTACHED	1
#define FSA9480_DETACHED	0

#define UART_SEL_SW	    58

struct fsa9480_platform_data {
	void (*cfg_gpio) (void);
	void (*otg_cb) (bool attached);
	void (*usb_cb) (bool attached);
	void (*uart_cb) (bool attached);
	void (*charger_cb) (bool attached);
	void (*jig_cb) (bool attached);
	void (*deskdock_cb) (bool attached);
	void (*cardock_cb) (bool attached);
	void (*mhl_cb) (bool attached);
	void (*reset_cb) (void);
	void (*set_init_flag) (void);
	void (*usb_power) (u8 on);
	int wakeup;
};

enum {
	SWITCH_PORT_AUTO = 0,
	SWITCH_PORT_USB,
	SWITCH_PORT_AUDIO,
	SWITCH_PORT_UART,
	SWITCH_PORT_VAUDIO,
	SWITCH_PORT_USB_OPEN,
	SWITCH_PORT_ALL_OPEN,	
};

extern void fsa9480_manual_switching(int path);
extern void fsa9480_otg_detach(void);
extern void fsa9480_check_device(void);  // Add for fsa9485 device check (Samsung)

#endif /* _FSA9480_H_ */
