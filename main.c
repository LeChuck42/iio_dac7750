/* 
 * Linux IIO driver for Texas Instruments DAC7750 12-Bit SPI DAC
 * 
 * Copyright (C) 2016  Matthias Seidel
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * 
 * ===============================================================================
 * 
 * What works:
 * - Basic I/O to DAC Registers
 * - sysfs interface using IIO
 * - setting range and output enable bits in control register
 * 
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "dac7750.h"

MODULE_AUTHOR("Matthias Seidel <matthias.seidel@ametek.com>");
MODULE_DESCRIPTION("Texas Instruments DAC7750");
MODULE_LICENSE("GPL v2");

#define DW_SPI_WORKAROUND

struct dac7750_state {
	struct spi_device		*spi;
	unsigned int			ctrl;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		__be32 d32;
		u8 d8[4];
	} data[2] ____cacheline_aligned;
};

enum dac7750_type {
	ID_DAC7750
};

static struct iio_chan_spec dac_channel[] = {
	{
		.type = IIO_CURRENT,
		.indexed = 0,
		.output = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_CALIBSCALE) |
			BIT(IIO_CHAN_INFO_CALIBBIAS),
		.scan_type = {
			.sign = 'u',
			.realbits = 12,
			.storagebits = 16,
			.shift = 4,
		}
	}
};


static int dac7750_write_unlocked(struct iio_dev *indio_dev,
	unsigned int addr, unsigned int val)
{
	struct dac7750_state *st = iio_priv(indio_dev);

	val |= DAC7750_ADDR(addr);
	st->data[0].d32 = cpu_to_be32(val);
	
	return spi_write(st->spi, &st->data[0].d8[1], 3);
}

static int dac7750_write(struct iio_dev *indio_dev, unsigned int addr,
	unsigned int val)
{
	int ret;

	mutex_lock(&indio_dev->mlock);
	ret = dac7750_write_unlocked(indio_dev, addr, val);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int dac7750_read(struct iio_dev *indio_dev, unsigned int addr)
{
	struct dac7750_state *st = iio_priv(indio_dev);
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[1],
			.len = 3,
			.cs_change = 1,
		}, {
			.rx_buf = &st->data[1].d8[1],
			.len = 3,
		},
	};

	mutex_lock(&indio_dev->mlock);

	st->data[0].d32 = cpu_to_be32(DAC7750_ADDR(DAC7750_ADDR_READ) | addr);
	st->data[1].d32 = cpu_to_be32(DAC7750_ADDR(DAC7750_ADDR_NOP));

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret >= 0)
		ret = be32_to_cpu(st->data[1].d32) & 0xffff;

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t dac7750_read_ctrl(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct dac7750_state *st = iio_priv(indio_dev);

	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	u32 mask  = (this_attr->address >> 8) & 0xFFFF;
	u32 shift = (this_attr->address     ) & 0xFF;

	return sprintf(buf, "%u\n", (st->ctrl & mask)>>shift);
}

static int dac7750_update_ctrl(struct iio_dev *indio_dev, unsigned int set,
	unsigned int clr)
{
	struct dac7750_state *st = iio_priv(indio_dev);
	unsigned int ret;

	mutex_lock(&indio_dev->mlock);

	st->ctrl &= ~clr;
	st->ctrl |= set;

	ret = dac7750_write_unlocked(indio_dev, DAC7750_ADDR_WR_CTRL, st->ctrl);

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t dac7750_write_ctrl(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);

	int ret;
	u32 val;

	u32 mask  = (this_attr->address >> 8) & 0xFFFF;
	u32 shift = (this_attr->address     ) & 0xFF;

	ret = kstrtou32(buf, 10, &val);
	if (ret)
		return ret;

	val = (val<<shift) & mask;
	ret = dac7750_update_ctrl(indio_dev, val, mask);

	return ret ? ret : len;
}

static ssize_t dac7750_write_range(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);

	int ret;

	if (strncmp("4_20", buf, 4) == 0)
		ret = dac7750_update_ctrl(indio_dev,
			DAC7750_RANGE_4_20<<DAC7750_SHIFT_CTRL_RANGE,
			DAC7750_MASK_CTRL_RANGE);
	else if (strncmp("0_20", buf, 4) == 0)
		ret = dac7750_update_ctrl(indio_dev,
			DAC7750_RANGE_0_20<<DAC7750_SHIFT_CTRL_RANGE,
			DAC7750_MASK_CTRL_RANGE);
	else if (strncmp("0_24", buf, 4) == 0)
		ret = dac7750_update_ctrl(indio_dev,
			DAC7750_RANGE_0_24<<DAC7750_SHIFT_CTRL_RANGE,
			DAC7750_MASK_CTRL_RANGE);
	else
		return -EINVAL;

	if (ret)
		return ret;

	return len;
}

static ssize_t dac7750_read_range(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct dac7750_state *st = iio_priv(indio_dev);

	unsigned int range = (st->ctrl & DAC7750_MASK_CTRL_RANGE)>>DAC7750_SHIFT_CTRL_RANGE;

	switch (range)
	{
		case DAC7750_RANGE_4_20: return sprintf(buf, "4_20\n");
		case DAC7750_RANGE_0_20: return sprintf(buf, "0_20\n");
		case DAC7750_RANGE_0_24: return sprintf(buf, "0_24\n");
		default:                 return sprintf(buf, "undefined (%u)\n", range);
	}
}

static IIO_DEVICE_ATTR(r_ext,
			S_IRUGO | S_IWUSR,
			dac7750_read_ctrl,
			dac7750_write_ctrl,
			DAC7750_IIO_ADDRESS(DAC7750_MASK_CTRL_REXT, DAC7750_SHIFT_CTRL_REXT));

static IIO_DEVICE_ATTR(out_current_en,
			S_IRUGO | S_IWUSR,
			dac7750_read_ctrl,
			dac7750_write_ctrl,
			DAC7750_IIO_ADDRESS(DAC7750_MASK_CTRL_OUTEN, DAC7750_SHIFT_CTRL_OUTEN));

static IIO_DEVICE_ATTR(range,
			S_IRUGO | S_IWUSR,
			dac7750_read_range,
			dac7750_write_range, 0);


static struct attribute *dac7750_attributes[] = {
	&iio_dev_attr_r_ext.dev_attr.attr,
	&iio_dev_attr_out_current_en.dev_attr.attr,
	&iio_dev_attr_range.dev_attr.attr,
	NULL
};


static int dac7750_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret;

	switch (m) {
	case IIO_CHAN_INFO_RAW:
		ret = dac7750_read(indio_dev, DAC7750_READ_DATA);
		if (ret < 0)
			return ret;
		*val = ret >> chan->scan_type.shift;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = (short)dac7750_read(indio_dev, DAC7750_READ_DAC_ZERO);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = (unsigned short)dac7750_read(indio_dev, DAC7750_READ_DAC_GAIN);
		if (ret < 0)
			return ret;
		*val = ret;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int dac7750_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val >= (1 << chan->scan_type.realbits) || val < 0)
			return -EINVAL;

		return dac7750_write(indio_dev, DAC7750_ADDR_WR_REG, val << chan->scan_type.shift);

	case IIO_CHAN_INFO_CALIBBIAS:
		if (val > 0x7FFF || val < -0x8000)
			return -EINVAL;

		return dac7750_write(indio_dev, DAC7750_ADDR_WR_DAC_ZERO, val);

	case IIO_CHAN_INFO_CALIBSCALE:
		if (val >= 0xFFFF || val < 0)
			return -EINVAL;

		return dac7750_write(indio_dev, DAC7750_ADDR_WR_DAC_GAIN, val);

	default:
		break;
	}

	return -EINVAL;
}


static const struct attribute_group dac7750_attribute_group = {
	.attrs = dac7750_attributes,
};


static const struct iio_info dac7750_info = {
	.read_raw = dac7750_read_raw,
	.write_raw = dac7750_write_raw,
	.attrs = &dac7750_attribute_group,
	.driver_module = THIS_MODULE,
};

static int dac7750_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct dac7750_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL) {
		dev_err(&spi->dev, "Failed to allocate iio device\n");
		return  -ENOMEM;
	}

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &dac7750_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = 1;
	indio_dev->channels = dac_channel;

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "Failed to register iio device: %d\n", ret);
		return ret;
	}
	
#ifdef DW_SPI_WORKAROUND
	spi->mode = SPI_MODE_3; // workaround SS going high inbetween bytes
#endif

	dac7750_update_ctrl(indio_dev, 0, -1);

	return 0;
}

static int dac7750_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	
	// disable output
	dac7750_update_ctrl(indio_dev, 0, -1);
	iio_device_unregister(indio_dev);

	return 0;
}



static const struct spi_device_id dac7750_ids[] = {
	{ "dac7750", ID_DAC7750 },
	{}
};
MODULE_DEVICE_TABLE(spi, dac7750_ids);

static struct spi_driver dac7750_driver = {
	.driver = {
		   .name = "dac7750",
	},
	.probe = dac7750_probe,
	.remove = dac7750_remove,
	.id_table = dac7750_ids,
};

module_spi_driver(dac7750_driver);
