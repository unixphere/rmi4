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
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include "rmi.h"

#define RMI_PAGE_SELECT_REGISTER 0xff
#define RMI_I2C_PAGE(addr) (((addr) >> 8) & 0xff)

struct rmi_i2c_data {
	struct mutex page_mutex;
	int page;
};

static irqreturn_t rmi_i2c_irq_thread(int irq, void *p)
{
	struct rmi_phys_device *phys = p;
	struct rmi_device *rmi_dev = phys->rmi_dev;
	struct rmi_driver *driver = rmi_dev->driver;

	if (driver && driver->irq_handler)
		driver->irq_handler(rmi_dev, irq);

	return IRQ_HANDLED;
}

/*
 * rmi_set_page - Set RMI page
 * @phys: The pointer to the rmi_phys_device struct
 * @page: The new page address.
 *
 * RMI devices have 16-bit addressing, but some of the physical
 * implementations (like SMBus) only have 8-bit addressing. So RMI implements
 * a page address at 0xff of every page so we can reliable page addresses
 * every 256 registers.
 *
 * The page_mutex lock must be held when this function is entered.
 *
 * Returns zero on success, non-zero on failure.
 */
static int rmi_set_page(struct rmi_phys_device *phys, unsigned int page)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	int rc;

	rc = i2c_smbus_write_byte_data(client, RMI_PAGE_SELECT_REGISTER, page);
	if (rc < 0) {
		dev_err(&client->dev,
			"%s: set page failed: %d.", __func__, rc);
		return rc;
	}
	data->page = page;
	return 0;
}

static int rmi_i2c_write_block(struct rmi_phys_device *phys, u16 addr, u8 *buf,
			       int len)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	int rc;

	mutex_lock(&data->page_mutex);

	if (RMI_I2C_PAGE(addr) != data->page) {
		rc = rmi_set_page(phys, RMI_I2C_PAGE(addr));
		if (rc < 0)
			goto exit;
	}

	rc = i2c_smbus_write_i2c_block_data(client, addr & 0xff,
					    sizeof(buf),
						buf);

exit:
	mutex_unlock(&data->page_mutex);
	return rc;
}

static int rmi_i2c_write(struct rmi_phys_device *phys, u16 addr, u8 data)
{
	int rc = rmi_i2c_write_block(phys, addr, &data, 1);
	return (rc < 0) ? rc : 0;
}

static int rmi_i2c_read_block(struct rmi_phys_device *phys, u16 addr, u8 *buf,
			      int len)
{
	struct i2c_client *client = to_i2c_client(phys->dev);
	struct rmi_i2c_data *data = phys->data;
	int rc;

	mutex_lock(&data->page_mutex);

	if (RMI_I2C_PAGE(addr) != data->page) {
		rc = rmi_set_page(phys, RMI_I2C_PAGE(addr));
		if (rc < 0)
			goto exit;
	}

	rc = i2c_smbus_read_i2c_block_data(client, addr & 0xff, len, buf);
exit:
	mutex_unlock(&data->page_mutex);
	return rc;
}

static int rmi_i2c_read(struct rmi_phys_device *phys, u16 addr, u8 *buf)
{
	int rc = rmi_i2c_read_block(phys, addr, buf, 1);
	return (rc < 0) ? rc : 0;
}

static int __devinit rmi_i2c_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct rmi_phys_device *rmi_phys;
	struct rmi_i2c_data *data;
	struct rmi_device_platform_data *pdata = client->dev.platform_data;
	int error;

	if (!pdata) {
		dev_err(&client->dev, "no platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	rmi_phys = kzalloc(sizeof(struct rmi_phys_device), GFP_KERNEL);
	if (!rmi_phys)
		return -ENOMEM;

	data = kzalloc(sizeof(struct rmi_i2c_data), GFP_KERNEL);
	if (!data) {
		error = -ENOMEM;
		goto err_phys;
	}

	rmi_phys->data = data;
	rmi_phys->dev = &client->dev;

	rmi_phys->write = rmi_i2c_write;
	rmi_phys->write_block = rmi_i2c_write_block;
	rmi_phys->read = rmi_i2c_read;
	rmi_phys->read_block = rmi_i2c_read_block;

	mutex_init(&data->page_mutex);

	if (pdata->gpio_config) {
		error = pdata->gpio_config();
		if (error < 0) {
			dev_err(&client->dev, "failed to setup irq %d\n",
				pdata->irq);
			goto err_data;
		}
	}

	error = rmi_set_page(rmi_phys, 0);
	if (error < 0)
		goto err_data;

	error = rmi_register_phys_device(rmi_phys);
	if (error) {
		dev_err(&client->dev, "failed to register physical driver\n");
		goto err_data;
	}
	i2c_set_clientdata(client, rmi_phys);

	if (pdata->irq > 0) {
		int irq_flags =
			(pdata->irq_polarity == RMI_IRQ_ACTIVE_HIGH) ?
			IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;

		error = request_threaded_irq(gpio_to_irq(pdata->irq),
				NULL, rmi_i2c_irq_thread, irq_flags,
				dev_name(&client->dev), rmi_phys);
		if (error < 0) {
			dev_err(&client->dev,
				"request_threaded_irq failed %d\n",
				pdata->irq);
			goto err_unregister;
		}
	}

	dev_info(&client->dev, "registered rmi i2c driver\n");
	return 0;

err_unregister:
	rmi_unregister_phys_device(rmi_phys);
err_data:
	kfree(data);
err_phys:
	kfree(rmi_phys);
	return error;
}

static int __devexit rmi_i2c_remove(struct i2c_client *client)
{
	struct rmi_phys_device *phys = i2c_get_clientdata(client);

	rmi_unregister_phys_device(phys);
	kfree(phys->data);
	kfree(phys);
	return 0;
}

static const struct i2c_device_id rmi_id[] = {
	{ "rmi", 0 },
	{ "rmi-i2c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rmi_id);

static struct i2c_driver rmi_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi-i2c"
	},
	.id_table	= rmi_id,
	.probe		= rmi_i2c_probe,
	.remove		= __devexit_p(rmi_i2c_remove),
};

static int __init rmi_i2c_init(void)
{
	return i2c_add_driver(&rmi_i2c_driver);
}

static void __exit rmi_i2c_exit(void)
{
	i2c_del_driver(&rmi_i2c_driver);
}

MODULE_AUTHOR("Eric Andersson <eric.andersson@unixphere.com>");
MODULE_DESCRIPTION("RMI i2c driver");
MODULE_LICENSE("GPL");

module_init(rmi_i2c_init);
module_exit(rmi_i2c_exit);
