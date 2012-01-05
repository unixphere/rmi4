/*
 * Copyright (c) 2011 Synaptics Incorporated
 * Copyright (c) 2011 Unixphere
 *
 * This driver adds support for generic RMI4 devices from Synpatics. It
 * implements the mandatory f01 RMI register and depends on the presence of
 * other required RMI functions.
 *
 * The RMI4 specification can be found here:
 * http://www.synaptics.com/sites/default/files/511-000136-01-Rev-E-RMI4%20Intrfacing%20Guide.pdf
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
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/list.h>
#include "rmi.h"
#include "rmi_driver.h"

#define PDT_START_SCAN_LOCATION	0x00e9
#define PDT_END_SCAN_LOCATION	0x0005

#define RMI4_END_OF_PDT(id) ((id) == 0x00 || (id) == 0xff)
#define RMI4_MAX_PAGE 0xff
#define RMI4_PAGE_SIZE 0x100

static void rmi_free_function_list(struct rmi_device *rmi_dev)
{
	struct rmi_function_container *entry, *n;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	list_for_each_entry_safe(entry, n, &data->rmi_functions.list, list)
		list_del(&entry->list);
}

static void rmi_driver_fh_add(struct rmi_device *rmi_dev,
			      struct rmi_function_handler *fh)
{
	struct rmi_function_container *entry;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fd.function_number == fh->func) {
			entry->fh = fh;
			if (fh->init)
				fh->init(entry);
		}

}

static void rmi_driver_fh_remove(struct rmi_device *rmi_dev,
				 struct rmi_function_handler *fh)
{
	struct rmi_function_container *entry;
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fd.function_number == fh->func) {
			if (fh->remove)
				fh->remove(entry);

			entry->fh = NULL;
		}
}

static int rmi_driver_irq_handler(struct rmi_device *rmi_dev, int irq)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct rmi_function_container *entry;
	u8 irq_status;
	int irq_bits;
	int error;

	error = rmi_read(rmi_dev, data->f01_fd.data_base_addr + 1, &irq_status);
	if (error < 0)
		return error;

	irq_bits = irq_status & (((1 << data->f01_num_of_irqs) - 1)
				<< data->f01_irq_pos);
	if (irq_bits)
		rmi_driver_f01_attention(rmi_dev,
					 irq_bits >> data->f01_irq_pos);

	list_for_each_entry(entry, &data->rmi_functions.list, list) {
		irq_bits = irq_status & (((1 << entry->num_of_irqs) - 1)
					<< entry->irq_pos);
		if (irq_bits && entry->fh && entry->fh->attention) {
			error = entry->fh->attention(entry,
					irq_bits >> entry->irq_pos);
			if (error < 0)
				pr_err(
				"%s: f%.2x attention handler failed: %d\n",
					 __func__, entry->fh->func, error);
		}
	}

	return 0;
}

static int rmi_driver_probe(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data;
	struct pdt_entry {
		u8 query_base_addr;
		u8 command_base_addr;
		u8 control_base_addr;
		u8 data_base_addr;
		u8 interrupt_source_count;
		u8 function_number;
	} pdt_entry;
	struct rmi_function_handler *fh;
	struct rmi_function_container *fc;
	int error;
	int i;
	int page;
	int irq_count = 0;
	struct device *dev = &rmi_dev->dev;
	bool done = false;

	data = kzalloc(sizeof(struct rmi_driver_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	rmi_set_driverdata(rmi_dev, data);

	INIT_LIST_HEAD(&data->rmi_functions.list);

	/* parse the PDT */
	for (page = 0; (page <= RMI4_MAX_PAGE) && !done; page++) {
		u16 page_start = RMI4_PAGE_SIZE * page;
		u16 pdt_start = page_start + PDT_START_SCAN_LOCATION;
		u16 pdt_end = page_start + PDT_END_SCAN_LOCATION;

		done = true;
		for (i = pdt_start; i >= pdt_end; i -= sizeof(pdt_entry)) {
			error = rmi_read_block(rmi_dev, i, (u8 *)&pdt_entry,
					       sizeof(pdt_entry));
			if (error != sizeof(pdt_entry))
				goto err_free_data;

			if (RMI4_END_OF_PDT(pdt_entry.function_number))
				break;

			done = false;

			/* f01 will be handled by rmi_driver */
			if (pdt_entry.function_number == 0x01) {
				data->f01_fd.query_base_addr =
				  pdt_entry.query_base_addr + page_start;
				data->f01_fd.command_base_addr =
				  pdt_entry.command_base_addr + page_start;
				data->f01_fd.control_base_addr =
				  pdt_entry.control_base_addr + page_start;
				data->f01_fd.data_base_addr =
				  pdt_entry.data_base_addr + page_start;
				data->f01_fd.function_number =
				  pdt_entry.function_number;
				data->f01_fd.interrupt_source_count =
				  pdt_entry.interrupt_source_count;
				data->f01_num_of_irqs =
				  pdt_entry.interrupt_source_count & 0x07;
				data->f01_irq_pos = irq_count;

				irq_count += data->f01_num_of_irqs;
				continue;
			}

			fc = kzalloc(sizeof(struct rmi_function_container),
				     GFP_KERNEL);
			if (!fc) {
				error = -ENOMEM;
				goto err_free_data;
			}

			fc->fd.query_base_addr =
				pdt_entry.query_base_addr + page_start;
			fc->fd.command_base_addr =
				pdt_entry.command_base_addr + page_start;
			fc->fd.control_base_addr =
				pdt_entry.control_base_addr + page_start;
			fc->fd.data_base_addr =
				pdt_entry.data_base_addr + page_start;
			fc->fd.function_number =
				pdt_entry.function_number;
			fc->fd.interrupt_source_count =
				pdt_entry.interrupt_source_count;

			fc->rmi_dev = rmi_dev;
			fc->num_of_irqs =
				pdt_entry.interrupt_source_count & 0x07;
			fc->irq_pos = irq_count;
			irq_count += fc->num_of_irqs;

			/*
			 * check if function handler exists, else it might be
			 * added later.
			 */
			fh = rmi_get_function_handler(
				pdt_entry.function_number);
			if (fh) {
				fc->fh = fh;
				error = fh->init(fc);
				if (error < 0) {
					dev_err(dev, "failed to init function f%.2x\n",
						pdt_entry.function_number);
					goto err_free_data;
				}
			}

			list_add_tail(&fc->list, &data->rmi_functions.list);
		}
	}

	if (data->f01_fd.function_number != 0x01) {
		dev_err(dev, "missing f01 function!\n");
		error = -EINVAL;
		goto err_free_data;
	}

	error = rmi_driver_f01_init(rmi_dev);
	if (error < 0)
		goto err_free_data;

	dev_info(dev, "connected RMI device manufacturer: %s product: %s\n",
		data->manufacturer_id == 1 ? "synaptics" : "unknown",
		data->product_id);

	return 0;

err_free_data:
	rmi_free_function_list(rmi_dev);
	kfree(data);
	return error;
}

#ifdef CONFIG_PM
static int rmi_driver_suspend(struct device *dev)
{
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;
	struct rmi_function_container *entry;
	int error;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->suspend) {
			error = entry->fh->suspend(entry);
			if (error < 0)
				return error;
		}

	return rmi_driver_f01_suspend(rmi_dev);
}

static int rmi_driver_resume(struct device *dev)
{
	struct rmi_device *rmi_dev;
	struct rmi_driver_data *data;
	struct rmi_function_container *entry;
	int error;

	rmi_dev = to_rmi_device(dev);
	data = rmi_get_driverdata(rmi_dev);

	error = rmi_driver_f01_resume(rmi_dev);
	if (error < 0)
		return error;

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->suspend) {
			error = entry->fh->resume(entry);
			if (error < 0)
				return error;
		}

	return 0;
}
#endif

static int __devexit rmi_driver_remove(struct rmi_device *rmi_dev)
{
	struct rmi_driver_data *data = rmi_get_driverdata(rmi_dev);
	struct rmi_function_container *entry;

	list_for_each_entry(entry, &data->rmi_functions.list, list)
		if (entry->fh && entry->fh->remove)
			entry->fh->remove(entry);

	rmi_free_function_list(rmi_dev);
	kfree(data);

	return 0;
}

static UNIVERSAL_DEV_PM_OPS(rmi_driver_pm, rmi_driver_suspend,
			    rmi_driver_resume, NULL);

static struct rmi_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "rmi-generic",
		.pm = &rmi_driver_pm
	},
	.probe = rmi_driver_probe,
	.irq_handler = rmi_driver_irq_handler,
	.fh_add = rmi_driver_fh_add,
	.fh_remove = rmi_driver_fh_remove,
	.remove = __devexit_p(rmi_driver_remove)
};

static int __init rmi_driver_init(void)
{
	return rmi_register_driver(&sensor_driver);
}

static void __exit rmi_driver_exit(void)
{
	rmi_unregister_driver(&sensor_driver);
}

module_init(rmi_driver_init);
module_exit(rmi_driver_exit);

MODULE_AUTHOR("Eric Andersson <eric.andersson@unixphere.com>");
MODULE_DESCRIPTION("RMI generic driver");
MODULE_LICENSE("GPL");
