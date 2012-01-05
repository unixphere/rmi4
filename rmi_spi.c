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
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include <linux/sched.h>
#include <linux/gpio.h>
#include "rmi.h"

#define RMI_PROTOCOL_VERSION_ADDRESS	0xa0fd
#define SPI_V2_UNIFIED_READ		0xc0
#define SPI_V2_WRITE			0x40
#define SPI_V2_PREPARE_SPLIT_READ	0xc8
#define SPI_V2_EXECUTE_SPLIT_READ	0xca

#define RMI_SPI_BLOCK_DELAY_US		65
#define RMI_SPI_BYTE_DELAY_US		65

#define DUMMY_READ_SLEEP_US		10
#define DUMMY_READ_PDT_START		0x00e9

struct spi_v2_data {
	bool split_read_pending;

	struct completion irq_comp;
};

static irqreturn_t rmi_spi_hard_irq(int irq, void *p)
{
	struct rmi_phys_device *phys = p;
	struct spi_v2_data *data = phys->data;

	if (data->split_read_pending) {
		complete(&data->irq_comp);
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t rmi_spi_irq_thread(int irq, void *p)
{
	struct rmi_phys_device *phys = p;
	struct rmi_device *rmi_dev = phys->rmi_dev;
	struct rmi_driver *driver = rmi_dev->driver;

	if (driver && driver->irq_handler)
		driver->irq_handler(rmi_dev, irq);

	return IRQ_HANDLED;
}

static int rmi_spi_xfer(struct rmi_phys_device *phys,
		    const u8 *txbuf, unsigned n_tx, u8 *rxbuf, unsigned n_rx)
{
	struct spi_device *client = to_spi_device(phys->dev);
	struct spi_v2_data *v2_data = phys->data;
	struct rmi_device_platform_data *pdata = phys->dev->platform_data;
	int status;
	struct spi_message message;
	struct spi_transfer *xfers;
	int total_bytes = n_tx + n_rx;
	u8 local_buf[total_bytes];
	int xfer_count = 0;
	int xfer_index = 0;
	int block_delay = n_rx > 0 ? pdata->spi_v2.block_delay_us : 0;
	int byte_delay = n_tx > 1 ? pdata->spi_v2.byte_delay_us : 0;

	if (v2_data->split_read_pending) {
		block_delay =
		    n_rx > 0 ? pdata->spi_v2.split_read_block_delay_us : 0;
		byte_delay =
		    n_tx > 1 ? pdata->spi_v2.split_read_byte_delay_us : 0;
	}

	if (n_tx)
		xfer_count += 1;

	if (n_rx) {
		if (byte_delay)
			xfer_count += n_rx;
		else
			xfer_count += 1;
	}

	xfers = kcalloc(xfer_count,
			    sizeof(struct spi_transfer), GFP_KERNEL);
	if (!xfers)
		return -ENOMEM;

	spi_message_init(&message);

	if (n_tx) {
		memset(&xfers[0], 0, sizeof(struct spi_transfer));
		xfers[0].len = n_tx;
		xfers[0].delay_usecs = block_delay;
		spi_message_add_tail(&xfers[0], &message);
		memcpy(local_buf, txbuf, n_tx);
		xfers[0].tx_buf = local_buf;
		xfer_index++;
	}
	if (n_rx) {
		if (byte_delay) {
			int buffer_offset = n_tx;
			for (; xfer_index < xfer_count; xfer_index++) {
				memset(&xfers[xfer_index], 0,
				       sizeof(struct spi_transfer));
				xfers[xfer_index].len = 1;
				xfers[xfer_index].delay_usecs = byte_delay;
				xfers[xfer_index].rx_buf =
				    local_buf + buffer_offset;
				buffer_offset++;
				spi_message_add_tail(&xfers[xfer_index],
						     &message);
			}
		} else {
			memset(&xfers[xfer_index], 0,
			       sizeof(struct spi_transfer));
			xfers[xfer_index].len = n_rx;
			xfers[xfer_index].rx_buf = local_buf + n_tx;
			spi_message_add_tail(&xfers[xfer_index], &message);
			xfer_index++;
		}
	}

	/* do the i/o */
	if (pdata->spi_v2.cs_assert) {
		status = pdata->spi_v2.cs_assert(
			pdata->spi_v2.cs_assert_data, true);
		if (!status) {
			pr_err("%s: Failed to assert CS.", __func__);
			/* nonzero means error */
			status = -1;
			goto error_exit;
		} else
			status = 0;
	}

	if (pdata->spi_v2.pre_delay_us)
		udelay(pdata->spi_v2.pre_delay_us);

	status = spi_sync(client, &message);

	if (pdata->spi_v2.post_delay_us)
		udelay(pdata->spi_v2.post_delay_us);

	if (pdata->spi_v2.cs_assert) {
		status = pdata->spi_v2.cs_assert(
			pdata->spi_v2.cs_assert_data, false);
		if (!status) {
			pr_err("%s: Failed to deassert CS.", __func__);
			/* nonzero means error */
			status = -1;
			goto error_exit;
		} else
			status = 0;
	}
	if (status == 0) {
		memcpy(rxbuf, local_buf + n_tx, n_rx);
		status = message.status;
	} else {
		pr_err("%s: spi_sync failed with error code %d.",
		       __func__, status);
	}

error_exit:
	kfree(xfers);
	return status;
}

static int rmi_spi_v2_write_block(struct rmi_phys_device *phys, u16 addr,
				  u8 *buf, int len)
{
	u8 txbuf[len + 4];
	int error;

	txbuf[0] = SPI_V2_WRITE;
	txbuf[1] = (addr >> 8) & 0x00FF;
	txbuf[2] = addr & 0x00FF;
	txbuf[3] = len;

	memcpy(&txbuf[4], buf, len);

	error = rmi_spi_xfer(phys, buf, len + 4, NULL, 0);
	if (error < 0)
		return error;

	return len;
}

static int rmi_spi_v2_write(struct rmi_phys_device *phys, u16 addr, u8 data)
{
	int error = rmi_spi_v2_write_block(phys, addr, &data, 1);

	return (error == 1) ? 0 : error;
}

static int rmi_spi_v1_write_block(struct rmi_phys_device *phys, u16 addr,
				  u8 *buf, int len)
{
	unsigned char txbuf[len + 2];
	int error;

	txbuf[0] = addr >> 8;
	txbuf[1] = addr;
	memcpy(txbuf+2, buf, len);

	error = rmi_spi_xfer(phys, txbuf, len + 2, NULL, 0);
	if (error < 0)
		return error;

	return len;
}

static int rmi_spi_v1_write(struct rmi_phys_device *phys, u16 addr, u8 data)
{
	int error = rmi_spi_v1_write_block(phys, addr, &data, 1);

	return (error == 1) ? 0 : error;
}

static int rmi_spi_v2_split_read_block(struct rmi_phys_device *phys, u16 addr,
				       u8 *buf, int len)
{
	struct spi_v2_data *data = phys->data;
	u8 txbuf[4];
	u8 rxbuf[len + 1]; /* one extra byte for read length */
	int error;

	txbuf[0] = SPI_V2_PREPARE_SPLIT_READ;
	txbuf[1] = (addr >> 8) & 0x00FF;
	txbuf[2] = addr & 0x00ff;
	txbuf[3] = len;

	data->split_read_pending = true;

	error = rmi_spi_xfer(phys, txbuf, 4, NULL, 0);
	if (error < 0) {
		data->split_read_pending = false;
		return error;
	}

	wait_for_completion(&data->irq_comp);

	txbuf[0] = SPI_V2_EXECUTE_SPLIT_READ;
	txbuf[1] = 0;

	error = rmi_spi_xfer(phys, txbuf, 2, rxbuf, len + 1);
	data->split_read_pending = false;
	if (error < 0)
		return error;

	/* first byte is length */
	if (rxbuf[0] != len)
		return -EIO;

	memcpy(buf, rxbuf + 1, len);

	return len;
}

static int rmi_spi_v2_read_block(struct rmi_phys_device *phys, u16 addr,
				 u8 *buf, int len)
{
	u8 txbuf[4];
	int error;

	txbuf[0] = SPI_V2_UNIFIED_READ;
	txbuf[1] = (addr >> 8) & 0x00FF;
	txbuf[2] = addr & 0x00ff;
	txbuf[3] = len;

	error = rmi_spi_xfer(phys, txbuf, 4, buf, len);
	if (error < 0)
		return error;

	return len;
}

static int rmi_spi_v2_read(struct rmi_phys_device *phys, u16 addr, u8 *buf)
{
	int error = rmi_spi_v2_read_block(phys, addr, buf, 1);

	return (error == 1) ? 0 : error;
}

static int rmi_spi_v1_read_block(struct rmi_phys_device *phys, u16 addr,
				 u8 *buf, int len)
{
	u8 txbuf[2];
	int error;

	txbuf[0] = (addr >> 8) | 0x80;
	txbuf[1] = addr;

	error = rmi_spi_xfer(phys, txbuf, 2, buf, len);
	if (error < 0)
		return error;

	return len;
}

static int rmi_spi_v1_read(struct rmi_phys_device *phys, u16 addr, u8 *buf)
{
	int error = rmi_spi_v1_read_block(phys, addr, buf, 1);

	return (error == 1) ? 0 : error;
}

static int rmi_spi_check_device(struct rmi_phys_device *rmi_phys)
{
	u8 buf[6];
	int error;
	int i;

	error = rmi_spi_v1_read_block(rmi_phys, DUMMY_READ_PDT_START,
				      buf, sizeof(buf));
	if (error < 0) {
		dev_err(rmi_phys->dev, "dummy read failed with %d.\n", error);
		return error;
	}
	udelay(DUMMY_READ_SLEEP_US);

	error = rmi_spi_v1_read_block(rmi_phys, DUMMY_READ_PDT_START,
				      buf, sizeof(buf));
	if (error < 0) {
		dev_err(rmi_phys->dev, "probe read failed with %d.\n", error);
		return error;
	}
	for (i = 0; i < sizeof(buf); i++) {
		if (buf[i] != 0x00 && buf[i] != 0xff)
			return 0;
	}

	dev_err(rmi_phys->dev, "probe read returned invalid block.\n");
	return -ENODEV;
}

static int __devinit rmi_spi_probe(struct spi_device *spi)
{
	struct rmi_phys_device *rmi_phys;
	struct spi_v2_data *data;
	struct rmi_device_platform_data *pdata = spi->dev.platform_data;
	u8 buf[2];
	int error;

	if (!pdata) {
		dev_err(&spi->dev, "no platform data\n");
		return -EINVAL;
	}

	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX)
		return -EINVAL;

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_3;
	error = spi_setup(spi);
	if (error < 0) {
		dev_err(&spi->dev, "spi_setup failed!\n");
		return error;
	}

	rmi_phys = kzalloc(sizeof(struct rmi_phys_device), GFP_KERNEL);
	if (!rmi_phys)
		return -ENOMEM;

	data = kzalloc(sizeof(struct spi_v2_data), GFP_KERNEL);
	if (!data) {
		error = -ENOMEM;
		goto err_phys;
	}

	rmi_phys->data = data;
	rmi_phys->dev = &spi->dev;

	rmi_phys->write		= rmi_spi_v1_write;
	rmi_phys->write_block	= rmi_spi_v1_write_block;
	rmi_phys->read		= rmi_spi_v1_read;
	rmi_phys->read_block	= rmi_spi_v1_read_block;

	dev_set_drvdata(&spi->dev, rmi_phys);

	pdata->spi_v2.block_delay_us = pdata->spi_v2.block_delay_us ?
			pdata->spi_v2.block_delay_us : RMI_SPI_BLOCK_DELAY_US;
	pdata->spi_v2.byte_delay_us = pdata->spi_v2.byte_delay_us ?
			pdata->spi_v2.byte_delay_us : RMI_SPI_BYTE_DELAY_US;
	pdata->spi_v2.split_read_block_delay_us =
			pdata->spi_v2.split_read_block_delay_us ?
			pdata->spi_v2.split_read_block_delay_us :
			RMI_SPI_BLOCK_DELAY_US;
	pdata->spi_v2.split_read_byte_delay_us =
			pdata->spi_v2.split_read_byte_delay_us ?
			pdata->spi_v2.split_read_byte_delay_us :
			RMI_SPI_BYTE_DELAY_US;

	if (pdata->gpio_config) {
		error = pdata->gpio_config();
		if (error < 0) {
			dev_err(&spi->dev, "failed to setup irq %d\n",
				pdata->irq);
			goto err_data;
		}
	}

	error = rmi_spi_check_device(rmi_phys);
	if (error < 0)
		goto err_data;

	/* check if this is an SPI v2 device */
	error = rmi_spi_v1_read_block(rmi_phys, RMI_PROTOCOL_VERSION_ADDRESS,
				      buf, 2);
	if (error < 0) {
		dev_err(&spi->dev, "failed to get SPI version number!\n");
		goto err_data;
	}
	dev_dbg(&spi->dev, "SPI version is %d", buf[0]);

	if (buf[0] == 1) {
		/* SPIv2 */
		rmi_phys->write		= rmi_spi_v2_write;
		rmi_phys->write_block	= rmi_spi_v2_write_block;
		rmi_phys->read		= rmi_spi_v2_read;

		if (pdata->irq > 0) {
			init_completion(&data->irq_comp);
			rmi_phys->read_block = rmi_spi_v2_split_read_block;
		} else {
			rmi_phys->read_block = rmi_spi_v2_read_block;
		}
	} else if (buf[0] != 0) {
		dev_err(&spi->dev, "Unrecognized SPI version %d.\n", buf[0]);
		error = -ENODEV;
		goto err_data;
	}

	error = rmi_register_phys_device(rmi_phys);
	if (error) {
		dev_err(&spi->dev, "failed to register physical driver\n");
		goto err_data;
	}

	if (pdata->irq > 0) {
		int irq_flags = (pdata->irq_polarity == RMI_IRQ_ACTIVE_HIGH) ?
				 IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING;

		error = request_threaded_irq(gpio_to_irq(pdata->irq),
				rmi_spi_hard_irq, rmi_spi_irq_thread, irq_flags,
				dev_name(&spi->dev), rmi_phys);
		if (error < 0) {
			dev_err(&spi->dev, "request_threaded_irq failed %d\n",
				pdata->irq);
			goto err_unregister;
		}
	}

	dev_info(&spi->dev, "registered RMI SPI driver\n");
	return 0;

err_unregister:
	rmi_unregister_phys_device(rmi_phys);
err_data:
	kfree(data);
err_phys:
	kfree(rmi_phys);
	return error;
}

static int __devexit rmi_spi_remove(struct spi_device *spi)
{
	struct rmi_phys_device *phys = dev_get_drvdata(&spi->dev);

	rmi_unregister_phys_device(phys);
	kfree(phys->data);
	kfree(phys);
	return 0;
}

static const struct spi_device_id rmi_id[] = {
	{ "rmi", 0 },
	{ "rmi-spi", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, rmi_id);

static struct spi_driver rmi_spi_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "rmi-spi",
	},
	.id_table	= rmi_id,
	.probe		= rmi_spi_probe,
	.remove		= __devexit_p(rmi_spi_remove),
};

static int __init rmi_spi_init(void)
{
	return spi_register_driver(&rmi_spi_driver);
}

static void __exit rmi_spi_exit(void)
{
	spi_unregister_driver(&rmi_spi_driver);
}

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("RMI SPI driver");
MODULE_LICENSE("GPL");

module_init(rmi_spi_init);
module_exit(rmi_spi_exit);
