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
#ifndef _RMI_H
#define _RMI_H
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/interrupt.h>

enum rmi_irq_polarity {
	RMI_IRQ_ACTIVE_HIGH,
	RMI_IRQ_ACTIVE_LOW
};

/**
 * struct rmi_f11_axis_alignmen - target axis alignment
 * @swap_axes: set to TRUE if desired to swap x- and y-axis
 * @flip_x: set to TRUE if desired to flip direction on x-axis
 * @flip_y: set to TRUE if desired to flip direction on y-axis
 */
struct rmi_f11_2d_axis_alignment {
	bool swap_axes;
	bool flip_x;
	bool flip_y;
};

/**
 * RMI F11 - function control register parameters
 * Each register that has a specific bit-field setup has an accompanied
 * register definition so that the setting can be chosen as a one-word
 * register setting or per-bit setting.
 */
union rmi_f11_2d_ctrl0 {
	struct {
		u8 reporting_mode:3;
		u8 abs_pos_filt:1;
		u8 rel_pos_filt:1;
		u8 rel_ballistics:1;
		u8 dribble:1;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl1 {
	struct {
		u8 palm_detect_thres:4;
		u8 motion_sensitivity:2;
		u8 man_track_en:1;
		u8 man_tracked_finger:1;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl6__7 {
	struct {
		u16 sensor_max_x_pos:12;
	};
	u8 regs[2];
};

union rmi_f11_2d_ctrl8__9 {
	struct {
		u16 sensor_max_y_pos:12;
	};
	u8 regs[2];
};

union rmi_f11_2d_ctrl10 {
	struct {
		u8 single_tap_int_enable:1;
		u8 tap_n_hold_int_enable:1;
		u8 double_tap_int_enable:1;
		u8 early_tap_int_enable:1;
		u8 flick_int_enable:1;
		u8 press_int_enable:1;
		u8 pinch_int_enable:1;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl11 {
	struct {
		u8 palm_detect_int_enable:1;
		u8 rotate_int_enable:1;
		u8 touch_shape_int_enable:1;
		u8 scroll_zone_int_enable:1;
		u8 multi_finger_scroll_int_enable:1;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl12 {
	struct {
		u8 sensor_map:7;
		u8 xy_sel:1;
	};
	u8 reg;
};

union rmi_f11_2d_ctrl14 {
	struct {
		u8 sens_adjustment:5;
		u8 hyst_adjustment:3;
	};
	u8 reg;
};

/* The configuation is controlled as per register which means that if a register
 * is allocated for ctrl configuration one must make sure that all the bits are
 * set accordingly for that particular register.
 */
struct  rmi_f11_2d_ctrl {
	union rmi_f11_2d_ctrl0		*ctrl0;
	union rmi_f11_2d_ctrl1		*ctrl1;
	u8				*ctrl2;
	u8				*ctrl3;
	u8				*ctrl4;
	u8				*ctrl5;
	union rmi_f11_2d_ctrl6__7	*ctrl6__7;
	union rmi_f11_2d_ctrl8__9	*ctrl8__9;
	union rmi_f11_2d_ctrl10		*ctrl10;
	union rmi_f11_2d_ctrl11		*ctrl11;
	union rmi_f11_2d_ctrl12		*ctrl12;
	u8				ctrl12_size;
	union rmi_f11_2d_ctrl14		*ctrl14;
	u8				*ctrl15;
	u8				*ctrl16;
	u8				*ctrl17;
	u8				*ctrl18;
	u8				*ctrl19;
};

struct rmi_device_platform_data_spi_v2 {
	int block_delay_us;
	int split_read_block_delay_us;
	int byte_delay_us;
	int split_read_byte_delay_us;
	int pre_delay_us;
	int post_delay_us;

	void *cs_assert_data;
	int (*cs_assert) (const void *cs_assert_data, const bool assert);
};

struct rmi_device_platform_data {
	char *driver_name;

	int irq;
	enum rmi_irq_polarity irq_polarity;
	int (*gpio_config)(void);

	struct rmi_device_platform_data_spi_v2 spi_v2;

	/* function handler pdata */
	struct rmi_f11_2d_ctrl *f11_ctrl;
	struct rmi_f11_2d_axis_alignment axis_align;
};

/**
 * struct rmi_function_descriptor - RMI function base addresses
 * @query_base_addr: The RMI Query base address
 * @command_base_addr: The RMI Command base address
 * @control_base_addr: The RMI Control base address
 * @data_base_addr: The RMI Data base address
 * @interrupt_source_count: The number of irqs this RMI function needs
 * @function_number: The RMI function number
 *
 * This struct is used when iterating the Page Description Table. The addresses
 * are 16-bit values to include the current page address.
 *
 */
struct rmi_function_descriptor {
	u16 query_base_addr;
	u16 command_base_addr;
	u16 control_base_addr;
	u16 data_base_addr;
	u8 interrupt_source_count;
	u8 function_number;
};

struct rmi_function_container;
struct rmi_device;

/**
 * struct rmi_function_handler - an RMI function handler
 * @func: The RMI function number
 * @init: Callback for RMI function init
 * @attention: Callback for RMI function attention
 * @suspend: Callback for function suspend
 * @resume: Callback for RMI function resume
 * @remove: Callback for RMI function removal
 *
 * This struct describes the interface of an RMI function. These are
 * registered to the bus using the rmi_register_function_driver() call.
 *
 */
struct rmi_function_handler {
	int func;
	int (*init)(struct rmi_function_container *fc);
	int (*attention)(struct rmi_function_container *fc, u8 irq_bits);
#ifdef CONFIG_PM
	int (*suspend)(struct rmi_function_container *fc);
	int (*resume)(struct rmi_function_container *fc);
#endif
	void (*remove)(struct rmi_function_container *fc);
};

/**
 * struct rmi_function_container - an element in a function handler list
 * @list: The list
 * @fd: The function descriptor of the RMI function
 * @rmi_dev: Pointer to the RMI device associated with this function container
 * @fh: The callbacks connected to this function
 * @num_of_irqs: The number of irqs needed by this function
 * @irq_pos: The position in the irq bitfield this function holds
 * @data: Private data pointer
 *
 */
struct rmi_function_container {
	struct list_head list;

	struct rmi_function_descriptor fd;
	struct rmi_device *rmi_dev;
	struct rmi_function_handler *fh;
	int num_of_irqs;
	int irq_pos;

	void *data;
};

/**
 * struct rmi_driver - represents an RMI driver
 * @driver: Device driver model driver
 * @probe: Callback for device probe
 * @remove: Callback for device removal
 * @shutdown: Callback for device shutdown
 * @irq_handler: Callback for handling irqs
 * @fh_add: Callback for function handler add
 * @fh_remove: Callback for function handler remove
 *
 * The RMI driver implements a driver on the RMI bus.
 *
 */
struct rmi_driver {
	struct device_driver driver;

	int (*probe)(struct rmi_device *rmi_dev);
	int (*remove)(struct rmi_device *rmi_dev);
	void (*shutdown)(struct rmi_device *rmi_dev);
	int (*irq_handler)(struct rmi_device *rmi_dev, int irq);
	void (*fh_add)(struct rmi_device *rmi_dev,
		       struct rmi_function_handler *fh);
	void (*fh_remove)(struct rmi_device *rmi_dev,
			  struct rmi_function_handler *fh);
};
#define to_rmi_driver(d) \
	container_of(d, struct rmi_driver, driver);

/**
 * struct rmi_phys_device - represent an RMI physical device
 * @dev: Pointer to the communication device, e.g. i2c or spi
 * @rmi_dev: Pointer to the RMI device
 * @write: Callback for write
 * @write_block: Callback for writing a block of data
 * @read: Callback for read
 * @read_block: Callback for reading a block of data
 * @data: Private data pointer
 *
 * The RMI physical device implements the glue between different communication
 * buses such as I2C and SPI.
 *
 */
struct rmi_phys_device {
	struct device *dev;
	struct rmi_device *rmi_dev;

	int (*write)(struct rmi_phys_device *phys, u16 addr, u8 data);
	int (*write_block)(struct rmi_phys_device *phys, u16 addr, u8 *buf,
			   int len);
	int (*read)(struct rmi_phys_device *phys, u16 addr, u8 *buf);
	int (*read_block)(struct rmi_phys_device *phys, u16 addr, u8 *buf,
			  int len);
	void *data;
};

/**
 * struct rmi_device - represents an RMI device
 * @dev: The device created for the RMI bus
 * @driver: Pointer to associated driver
 * @phys: Pointer to the physical interface
 *
 * This structs represent an RMI device.
 *
 */
struct rmi_device {
	struct device dev;

	struct rmi_driver *driver;
	struct rmi_phys_device *phys;
};
#define to_rmi_device(d) container_of(d, struct rmi_device, dev);
#define to_rmi_platform_data(d) ((d)->phys->dev->platform_data);

static inline void rmi_set_driverdata(struct rmi_device *d, void *data)
{
	dev_set_drvdata(&d->dev, data);
}

static inline void *rmi_get_driverdata(struct rmi_device *d)
{
	return dev_get_drvdata(&d->dev);
}

/**
 * rmi_read - RMI read byte
 * @d: Pointer to an RMI device
 * @addr: The address to read from
 * @buf: The read buffer
 *
 * Reads a byte of data using the underlaying physical protocol in to buf. It
 * returns zero or a negative error code.
 */
static inline int rmi_read(struct rmi_device *d, u16 addr, u8 *buf)
{
	return d->phys->read(d->phys, addr, buf);
}

/**
 * rmi_read_block - RMI read block
 * @d: Pointer to an RMI device
 * @addr: The start address to read from
 * @buf: The read buffer
 * @len: Length of the read buffer
 *
 * Reads a block of byte data using the underlaying physical protocol in to buf.
 * It returns the amount of bytes read or a negative error code.
 */
static inline int rmi_read_block(struct rmi_device *d, u16 addr, u8 *buf,
				 int len)
{
	return d->phys->read_block(d->phys, addr, buf, len);
}

/**
 * rmi_write - RMI write byte
 * @d: Pointer to an RMI device
 * @addr: The address to write to
 * @data: The data to write
 *
 * Writes a byte from buf using the underlaying physical protocol. It
 * returns zero or a negative error code.
 */
static inline int rmi_write(struct rmi_device *d, u16 addr, u8 data)
{
	return d->phys->write(d->phys, addr, data);
}

/**
 * rmi_write_block - RMI write block
 * @d: Pointer to an RMI device
 * @addr: The start address to write to
 * @buf: The write buffer
 * @len: Length of the write buffer
 *
 * Writes a block of byte data from buf using the underlaying physical protocol.
 * It returns the amount of bytes written or a negative error code.
 */
static inline int rmi_write_block(struct rmi_device *d, u16 addr, u8 *buf,
				  int len)
{
	return d->phys->write_block(d->phys, addr, buf, len);
}

/**
 * rmi_register_driver - register rmi driver
 * @driver: the driver to register
 *
 * This function registers an RMI driver to the RMI bus.
 */
int rmi_register_driver(struct rmi_driver *driver);

/**
 * rmi_unregister_driver - unregister rmi driver
 * @driver: the driver to unregister
 *
 * This function unregisters an RMI driver to the RMI bus.
 */
void rmi_unregister_driver(struct rmi_driver *driver);

/**
 * rmi_register_phys_device - register a physical device connection
 * @phys: the physical driver to register
 *
 * This function registers a physical driver to the RMI bus. These drivers
 * provide a communication layer for the drivers connected to the bus, e.g.
 * I2C, SPI and so on.
 */
int rmi_register_phys_device(struct rmi_phys_device *phys);

/**
 * rmi_unregister_phys_device - unregister a physical device connection
 * @phys: the physical driver to unregister
 *
 * This function unregisters a physical driver from the RMI bus.
 */
void rmi_unregister_phys_device(struct rmi_phys_device *phys);

/**
 * rmi_register_function_driver - register an RMI function driver
 * @fh: the function handler to register
 *
 * This function registers support for a new RMI function to the bus. All
 * drivers on the bus will be notified of the presence of the new function
 * driver.
 */
int rmi_register_function_driver(struct rmi_function_handler *fh);

/**
 * rmi_unregister_function_driver - unregister an RMI function driver
 * @fh: the function handler to unregister
 *
 * This function unregisters a RMI function from the RMI bus. All drivers on
 * the bus will be notified of the removal of a function driver.
 */
void rmi_unregister_function_driver(struct rmi_function_handler *fh);

/**
 * rmi_get_function_handler - get a pointer to specified RMI function
 * @id: the RMI function id
 *
 * This function gets the specified RMI function handler from the list of
 * supported functions.
 */
struct rmi_function_handler *rmi_get_function_handler(int id);

#endif
