/*
 * Copyright (c) 2016, Toradex AG
 * Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/kfifo.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

/* TODO: Clean up */
#define RPMSGIIO_OFFSET_REF_VDD 2

#define RPMSGIIO_RAW_READ		0
#define RPMSGIIO_CONT_READ		1

#define READ_MODE(x)	((x) << 4)
#define ADC_CHANNEL(x)  (x & 0xF)

#define VF610_ADC_CHAN(_idx, _chan_type) {			\
	.type = (_chan_type),					\
	.indexed = 1,						\
	.channel = (_idx),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_idx),			\
	.scan_type = {					\
		.sign = 'u',				\
		.realbits = 12,				\
		.storagebits = 16,			\
	},						\
}

struct rpmsgiio_state {
	struct rpmsg_channel *rpdev;
	struct device *dev;
	u32 m4_config;
	u32 raw_data;
	bool got_raw;
	int sampling_frequency ;
	wait_queue_head_t wait_list;
};

/* rpmsgiio_adc_channels - structure that holds information about the
   channels that are present on the adc */
static const struct iio_chan_spec rpmsgiio_adc_channels[] = {
	/* TODO */
#if 0
	VF610_ADC_CHAN(0, IIO_VOLTAGE),
	VF610_ADC_CHAN(1, IIO_VOLTAGE),
	VF610_ADC_CHAN(2, IIO_VOLTAGE),
	VF610_ADC_CHAN(3, IIO_VOLTAGE),
	VF610_ADC_CHAN(4, IIO_VOLTAGE),
	VF610_ADC_CHAN(5, IIO_VOLTAGE),
	VF610_ADC_CHAN(6, IIO_VOLTAGE),
	VF610_ADC_CHAN(7, IIO_VOLTAGE),
#endif
	VF610_ADC_CHAN(8, IIO_VOLTAGE),
	/* TODO */
#if 0
	VF610_ADC_CHAN(9, IIO_VOLTAGE),
	VF610_ADC_CHAN(10, IIO_VOLTAGE),
	VF610_ADC_CHAN(11, IIO_VOLTAGE),
	VF610_ADC_CHAN(12, IIO_VOLTAGE),
	VF610_ADC_CHAN(13, IIO_VOLTAGE),
	VF610_ADC_CHAN(14, IIO_VOLTAGE),
	VF610_ADC_CHAN(15, IIO_VOLTAGE),
#endif
};

#if 0 /* TODO */
/*
 * set_rpmsgiio_sampling_frequency - To set the sampling_frequency and save
 * for the current instance of the device into the rpmsgiio_state structure
 *
 * @st			current rpmsgiio state instance
 * @*val		pointer to the required sampling frequency
 *
 * Description - The function maps 3 pointers to the different regions of the
 * m4_config buffer. It then store the configuration data into appropriately
 * pointed regions. The frequency value is converted into:
 * the time period then to
 * number of m4 cycles
 * it is then divided into 3 parts, 2 of which are use for the clock on time
 * (cycle_after/before_sample) and the other (cycle_between_sample) is used for
 * clock off time.
 * These 3 values need to be odd (this is PRU's requirement). Finally the
 * resultant configuration value is used to get the resultant frequency, as the
 * m4 cant attain all the frequencies but a large number of frequencies. This
 * resultant frequency is what saved in the current state of the rpmsgiio
 */
static void set_rpmsgiio_sampling_frequency(struct rpmsgiio_state *st, int *val)
{

	u32 *cycle_between_sample = st->m4_config;
	u16 *cycle_before_sample = (u16 *)&st->m4_config[1];
	u16 *cycle_after_sample = ((u16*)&st->m4_config[1])+1;
	u32 time_period_ns;
	u32 m4_cycles;
	u32 remainder;

	printk(KERN_INFO "%s: %s\n", __FILE__, "set_m4_sampling_frequency");

	time_period_ns = 1000000000/(*val);
	m4_cycles = time_period_ns/5;
	m4_cycles -= 5;
	remainder = m4_cycles%4;
	m4_cycles = m4_cycles/4;
	*cycle_between_sample = 2*m4_cycles + (remainder == 3 ? 3 : 1 );
	if (m4_cycles%2) {
		*cycle_after_sample = m4_cycles;
		*cycle_before_sample = m4_cycles;
	} else {
		*cycle_after_sample = m4_cycles+1;
		*cycle_before_sample = m4_cycles-1;
	}

	st->sampling_frequency = 1000000000/((*cycle_between_sample +
			   *cycle_before_sample +
			   *cycle_after_sample + 5) * 5);
	pr_err("%u, %u, %u\n", *cycle_between_sample, *cycle_before_sample,
	       *cycle_after_sample);
	pr_err("Requested sampling freqeuency %u\n", *val);
	pr_err("Available sampling freqeuency %u\n", st->sampling_frequency);

}
#endif

/*
 * set_rpmsgiio_read_mode - To set read mode into the configuration data
 *
 * @st			current rpmsgiio state instance
 * @read_mode		which has to be set
 *
 * Description - The function maps the appropriate pointers to the configuration
 * buffer and uses them to place set the read mode in which the m4 need to be
 * activated. The function uses and configures last 32bits of the buffer. It
 * sets the read mode in the first bit of the those 32 bits and then sets the
 * enable bit, ie, the last bit in those 32bits.
 */
static void set_rpmsgiio_read_mode(struct rpmsgiio_state *st, bool read_mode)
{
	printk(KERN_INFO "%s: %s\n", __FILE__, "set_m4_read_mode");

	st->m4_config &= ~(0x10);
	st->m4_config |= READ_MODE(read_mode);
}

/*
 * get_rpmsgiio_read_mode - To get the read mode of the current rpmsgiio
 * device instance
 *
 * @st		The current instance of the rpmsgiio state structure
 *
 * Description - The function returns the read mode to which the beaglescop is
 * set. This is basically done by reading the associated bit of the m4_config
 * buffer. It returns boolean value.
 */
static bool get_rpmsgiio_read_mode(struct rpmsgiio_state *st)
{
	return ((st->m4_config & 0x10) >> 4);
}

/*
 * rpmsgiio_read_from_m4 - To start the PRUs in the required reading mode
 *
 * @indio_dev	pointer to the instance of iio device
 *
 * Description - The function starts the PRUs by sending the configuration data
 * that has been prepared by call to set_rpmsgiio_sampling_frequency() and
 * set_m4_read_mode() function calls. The message is then dispatched by the
 * rpmsg callback method. This function also checks if the required rpmsg device
 * has been released or not. If it has been released, the driver would return
 * with an error.
 *
 * In case the read_mode is RPMSGIIO_RAW_READ, this function waits for the
 * callback to interrupt, by using the wait_list and returns only after the
 * data from the callback has been saved into the raw_data field of the current
 * rpmsgiio_state structure.
 * If the read_mode is RPMSGIIO_CONT_READ, the function just configures the
 * PRUs and returns. The rpmsg callback method then pushes the data onto thes
 * iio_buffer.
 */
static int rpmsgiio_read_from_m4(struct iio_dev *indio_dev, int chan)
{
	int ret;
	struct rpmsgiio_state *st;

	printk(KERN_INFO "%s: %s\n", __FILE__, "rpmsgiio_read_from_m4");

	st = iio_priv(indio_dev);

	if (!st->rpdev) {
		dev_err(st->dev, "Required rpmsg device has been released\n");
		return -EINVAL;
	}

	st->m4_config |= 0x20;		/* Enable flag */
	st->m4_config |= chan;		/* Channel number */

	ret = rpmsg_send(st->rpdev, &st->m4_config, sizeof(u32));
	if (ret)
		dev_err(st->dev, "Failed sending config info to M4\n");

	if (get_rpmsgiio_read_mode(st) == RPMSGIIO_RAW_READ) {
		ret = wait_event_interruptible(st->wait_list, st->got_raw);
		if (ret)
			return -EINTR;
	}

	return 0;
}

/*
 * rpmsgiio_stop_sampling - to stop sampling process of the PRUS immediately
 */
static int rpmsgiio_stop_sampling_m4(struct iio_dev *indio_dev)
{
	int ret = 0;
	struct rpmsgiio_state *st;

	st = iio_priv(indio_dev);

	if (!st->rpdev) {
		dev_err(st->dev, "Required rpmsg device already released\n");
		return -EINVAL;
	}

	st->m4_config &= ~(0x20);

	ret = rpmsg_send(st->rpdev, &st->m4_config, sizeof(u32));
	if (ret)
		dev_err(st->dev, "failed to stop rpmsgiio sampling\n");

	return ret;
}

/*
 * rpmsgiio_buffer_postenable - function to do necessay work
 * just after the buffer gets enabled
 */
static int rpmsgiio_buffer_postenable(struct iio_dev *indio_dev)
{
	int ret, channel;
	struct rpmsgiio_state *st;

	channel = find_first_bit(indio_dev->active_scan_mask,
						indio_dev->masklength);

	printk(KERN_INFO "%s: %s\n", __FILE__,"postenable");

	st = iio_priv(indio_dev);

	set_rpmsgiio_read_mode(st, RPMSGIIO_CONT_READ);
	ret = rpmsgiio_read_from_m4(indio_dev, channel);

	return ret;
}

/*
 * rpmsgiio_buffer_predisable - function to do necessay work
 * just before the buffer gets disabled
 */
static int rpmsgiio_buffer_predisable(struct iio_dev *indio_dev)
{
	printk(KERN_INFO "%s: %s", __FILE__, "predisable\n");

	return rpmsgiio_stop_sampling_m4(indio_dev);
}

static const struct iio_buffer_setup_ops rpmsgiio_buffer_setup_ops = {
	.postenable = &rpmsgiio_buffer_postenable,
	.predisable = &rpmsgiio_buffer_predisable,
	.validate_scan_mask = &iio_validate_scan_mask_onehot,
};

static int rpmsgiio_read_raw(struct iio_dev *indio_dev,
               struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
       //int ret;
       struct rpmsgiio_state *st;
       printk(KERN_INFO "%s: %s\n", __FILE__, "read_raw");

       st = iio_priv(indio_dev);

       switch (mask) {
       case IIO_CHAN_INFO_RAW:
		//set_rpmsgiio_read_mode(st, RPMSGIIO_RAW_READ);
		//ret = rpmsgiio_read_from_m4(indio_dev, chan->channel);
		//if (ret) {
		//	dev_err(st->dev, "Couldn't read raw data\n");
		//	return -EINVAL;
		//}
	       //*val = st->raw_data;
		*val = 0;
	       return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
	       *val = 1;
	       return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
	       *val = - RPMSGIIO_OFFSET_REF_VDD;
	       return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
	       *val = st->sampling_frequency;
	       return IIO_VAL_INT;
        default:
                return -EINVAL;
       }
}

static int rpmsgiio_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,int val,
			       int val2, long mask)
{
	struct rpmsgiio_state *st;
	st = iio_priv(indio_dev);

	printk(KERN_INFO "%s: %s", __FILE__, "write_raw\n");

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		//set_rpmsgiio_sampling_frequency(st, &val);
		return 0;
	default:
		return -EINVAL;
	}
}

/* rpmsgiio_info - Structure contains constant data about the driver */
static const struct iio_info rpmsgiio_info = {
	.read_raw = rpmsgiio_read_raw,
	.write_raw = rpmsgiio_write_raw,
	.driver_module = THIS_MODULE,
};

/**
 * rpmsgiio_driver_cb() - function gets invoked each time the m4 sends some
 * data.
 *
 */
static void rpmsgiio_driver_cb(struct rpmsg_channel *rpdev, void *data,
				  int len, void *priv, u32 src)
{
	int count;
	u16 *dataw = data;
	struct rpmsgiio_state *st;
	struct iio_dev *indio_dev;

	indio_dev = dev_get_drvdata(&rpdev->dev);
	st = iio_priv(indio_dev);

	if (get_rpmsgiio_read_mode(st) == RPMSGIIO_RAW_READ) {
		printk(KERN_INFO "%s: %s", __FILE__, "callback - raw mode\n");
		st->raw_data = *(u32 *)data;
		st->got_raw = 1;
		wake_up_interruptible(&st->wait_list);
	} else if (get_rpmsgiio_read_mode(st) == RPMSGIIO_CONT_READ) {
		for (count = 0; count < len; count++) {
			iio_push_to_buffers(indio_dev, dataw + count);
		}
	}
}

/**
 * rpmsgiio_driver_probe() - function gets invoked when the rpmsg channel
 * as mentioned in the rpmsgiio_id table
 *
 * The function
 * - allocates space for the IIO device
 * - registers the device to the IIO subsystem
 * - exposes the sys entries according to the channels info
 */
static int rpmsgiio_driver_probe (struct rpmsg_channel *rpdev)
{
	int ret;
	struct iio_dev *indio_dev;
	struct rpmsgiio_state *st;
	struct rpmsg_device_id *id;
	struct iio_buffer *buffer;

	dev_info(&rpdev->dev, "New channel: 0x%x -> 0x%x!\n",
			rpdev->src, rpdev->dst);

	indio_dev = devm_iio_device_alloc(&rpdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	id = &rpdev->id;
	st = iio_priv(indio_dev);

	st->rpdev = rpdev;
	if (!st->rpdev) {
		dev_err(st->dev, "Required rpmsg device has been released\n");
		return -EINVAL;
	}

	st->dev = &rpdev->dev;

	dev_set_drvdata(st->dev, indio_dev);

	indio_dev->dev.parent = st->dev;
	indio_dev->name = id->name;
	indio_dev->info = &rpmsgiio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = rpmsgiio_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(rpmsgiio_adc_channels);

	ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
					NULL, &rpmsgiio_buffer_setup_ops);
	if (ret < 0) {
		dev_err(st->dev, "Couldn't initialise the buffer\n");
		return ret;
	}

	buffer = devm_iio_kfifo_allocate(&indio_dev->dev);
	if (!buffer)
		return -ENOMEM;

	iio_device_attach_buffer(indio_dev, buffer);

	init_waitqueue_head(&st->wait_list);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		pr_err("Failed to register with iio\n");
		return ret;
	}

	return 0;
}

/**
 * rpmsgiio_driver_remove() - function gets invoked when the rpmsg device is
 * removed
 */
static void rpmsgiio_driver_remove(struct rpmsg_channel *rpdev)
{
	struct iio_dev *indio_dev;

	indio_dev = dev_get_drvdata(&rpdev->dev);
	iio_device_free(indio_dev);
	dev_info(&rpdev->dev, "rpmsgiio driver is removed\n");
}

/* rpmsgiio_id - Structure that holds the channel name for which this driver
   should be probed */
static const struct rpmsg_device_id rpmsgiio_id_table[] = {
		{ .name = "rpmsg-openamp-demo-channel" },
		{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsgiio_id_table);

/* rpmsgiio_driver - The structure containing the pointers to read/write
   functions to send data to the m4 core */
static struct rpmsg_driver rpmsgiio_driver = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= rpmsgiio_id_table,
	.probe		= rpmsgiio_driver_probe,
	.callback	= rpmsgiio_driver_cb,
	.remove		= rpmsgiio_driver_remove,
};

/**
 * rpmsgiio_driver_init() : driver driver registration
 *
 * The initialization function gets invoked when the driver is loaded. The
 * function registers itself on the virtio_rpmsg_bus and it gets invoked when
 * the m4 creates a channel named as in the rpmsgiio_id structure.
 */
static int __init rpmsgiio_driver_init(void)
{
	return register_rpmsg_driver(&rpmsgiio_driver);
}

/**
 * rpmsgiio_driver_exit() - function invoked when the driver is unloaded
 */
static void __exit rpmsgiio_driver_exit(void)
{
	unregister_rpmsg_driver (&rpmsgiio_driver);
}

module_init(rpmsgiio_driver_init);
module_exit(rpmsgiio_driver_exit);

MODULE_AUTHOR("Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>");
MODULE_DESCRIPTION("VF610 RPMSG IIO Driver");
MODULE_LICENSE("GPL v2");
