/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef _RMI_DRIVER_H
#define _RMI_DRIVER_H

#define RMI_PRODUCT_ID_LENGTH 10

struct rmi_driver_data {
	struct rmi_function_container rmi_functions;

	struct rmi_function_descriptor f01_fd;
	int f01_num_of_irqs;
	int f01_irq_pos;

	u8 manufacturer_id;
	/* product id + null termination */
	u8 product_id[RMI_PRODUCT_ID_LENGTH + 1];
};

int rmi_driver_f01_init(struct rmi_device *rmi_dev);
int rmi_driver_f01_attention(struct rmi_device *rmi_dev, u8 irq_bits);
#ifdef CONFIG_PM
int rmi_driver_f01_suspend(struct rmi_device *rmi_dev);
int rmi_driver_f01_resume(struct rmi_device *rmi_dev);
#endif

#endif
