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
#include <linux/kernel.h>
#include "rmi.h"
#include "rmi_driver.h"

/* control register bits */

#define F01_SLEEP_OFFSET 0
#define F01_SLEEP_MASK	(0x03 << F01_SLEEP_OFFSET)

enum f01_sleep_modes {
	normal_operation = 0x00 << F01_SLEEP_OFFSET,
	sensor_sleep = 0x01 << F01_SLEEP_OFFSET,
	reserved_1 = 0x02 << F01_SLEEP_OFFSET,
	reserved_2 = 0x03 << F01_SLEEP_OFFSET,
};

#define F01_CONFIGURED	(1 << 7)

int rmi_driver_f01_init(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	int error;
	u8 temp;

	/* set F01_CONFIGURED bit */
	error = rmi_read(rmi_dev, data->f01_fd.control_base_addr, &temp);
	if (error < 0)
		return error;

	error = rmi_write(rmi_dev, data->f01_fd.control_base_addr,
			  temp | F01_CONFIGURED);
	if (error < 0)
		return error;

	/* enable irqs and then read them to clear them */
	error = rmi_write(rmi_dev, data->f01_fd.control_base_addr + 1, 0xff);
	if (error < 0)
		return error;

	error = rmi_read(rmi_dev, data->f01_fd.data_base_addr + 1, &temp);
	if (error < 0)
		return error;

	error = rmi_read(rmi_dev, data->f01_fd.query_base_addr,
		&data->manufacturer_id);
	if (error < 0)
		return error;

	error = rmi_read_block(rmi_dev, data->f01_fd.query_base_addr + 11,
		data->product_id, RMI_PRODUCT_ID_LENGTH);
	if (error != RMI_PRODUCT_ID_LENGTH)
		return error;

	data->product_id[RMI_PRODUCT_ID_LENGTH] = '\0';

	error = rmi_read(rmi_dev, data->f01_fd.data_base_addr,
		&temp);
	if (error < 0)
		return error;

	if (temp & F01_CONFIGURED) {
		dev_err(&rmi_dev->dev,
			"Device reset during configuration process!\n");
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_PM
int rmi_driver_f01_suspend(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	u8 dev_control;
	int error;

	error = rmi_read(rmi_dev, data->f01_fd.control_base_addr,
			 &dev_control);
	if (error < 0)
		return error;

	dev_control = (dev_control & ~F01_SLEEP_MASK) | sensor_sleep;

	return rmi_write(rmi_dev, data->f01_fd.control_base_addr,
			 dev_control);
}

int rmi_driver_f01_resume(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	u8 dev_control;
	int error;

	error = rmi_read(rmi_dev, data->f01_fd.control_base_addr,
			 &dev_control);
	if (error < 0)
		return error;

	dev_control = (dev_control & ~F01_SLEEP_MASK) | normal_operation;

	return rmi_write(rmi_dev, data->f01_fd.control_base_addr,
			 dev_control);
}
#endif

int rmi_driver_f01_attention(struct rmi_device *rmi_dev, u8 irq_bits)
{
	return 0;
}
