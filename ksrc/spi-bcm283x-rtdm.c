/**
 * Copyright (C) 2016 Nicolas Schurando <schurann@ext.essilor.com>
 * Copyright (C) 2017 Piotr Piórkowski <qba100@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/* Self header */
#include "../include/spi-bcm283x-rtdm.h"

/* Bypass CLion syntax checker */
#ifdef __CDT_PARSER__
    #define __init
	#define __exit
    #define MODULE_VERSION(x)
    #define MODULE_DESCRIPTION(x)
    #define MODULE_AUTHOR(x)
    #define MODULE_LICENSE(x)
    #define module_init(x)
    #define module_exit(x)
#endif /* __CDT_PARSER__ */


/**
 * This structure enumerates the two RTDM devices created for SPI0.
 */
static struct rtdm_device spi_bcm283x_devices[SPI_BCM283X_RTDM_DEVICES_NUMBER];
static spi_bcm283x_context_t * bcm283x_spi_rtdm_open_devices[SPI_BCM283X_RTDM_DEVICES_NUMBER];


/**
 * Open handler. Note: opening a named device instance always happens from secondary mode.
 * @param[in] context The context associated with the device.
 * @return 0 on success. On failure, a negative error code is returned.
 */
static int bcm283x_spi_rtdm_set_default_config(spi_bcm283x_context_t *context, int chip_select){
    /* Retrieve context */
    if(context == NULL )
        return -1;


    context->config.bit_order = BCM2835_SPI_BIT_ORDER_MSBFIRST;
    context->config.data_mode = BCM2835_SPI_MODE1;
    context->config.clock_divider = BCM2835_SPI_CLOCK_DIVIDER_8;
    context->config.chip_select_polarity = LOW;
    context->config.chip_select = BCM2835_SPI_CS0;
    bcm2835_spi_chipSelect(context->config.chip_select);
    bcm2835_spi_setChipSelectPolarity(context->config.chip_select, context->config.chip_select_polarity);
    return 0;
}

/**
 * Open handler. Note: opening a named device instance always happens from secondary mode.
 * @param[in] fd File descriptor associated with opened device instance.
 * @param[in] oflags Open flags as passed by the user.
 * @return 0 on success. On failure, a negative error code is returned.
 */
static int bcm283x_spi_rtdm_open(struct rtdm_fd *fd, int oflags) {
	if(fd == NULL)
		return -1;
	spi_bcm283x_context_t * context = (spi_bcm283x_context_t *) rtdm_fd_to_private(fd);
    if(context->device_used == 1){
        printk(KERN_ERR "Device /dev/rtdm/spidev0.%d is busy" , fd->minor);
        return EBUSY;
    }
    context->device_used = 1;
    bcm283x_spi_rtdm_open_devices[fd->minor] = context;
	return bcm283x_spi_rtdm_set_default_config(context, rtdm_fd_minor(fd));
}



/**
 * Close handler. Note: closing a device instance always happens from secondary mode.
 * @param[in] fd File descriptor associated with opened device instance.
 * @return 0 on success. On failure return a negative error code.
 */
static int  bcm283x_spi_rtdm_close(struct rtdm_fd *fd) {
    spi_bcm283x_context_t * context = (spi_bcm283x_context_t *) rtdm_fd_to_private(fd);
    context->device_used = 0;
	return 0;

}


/**
 * Read from the device.
 * @param[in] context The context associated with the device.
 * @param[out] buf Input buffer as passed by the user.
 * @param[in] size Number of bytes the user requests to read.
 * @return On success, the number of bytes read. On failure return either -ENOSYS, to request that this handler be called again from the opposite realtime/non-realtime context, or another negative error code.
 */
static ssize_t bcm283x_spi_rtdm_read_rt_using_context(spi_bcm283x_context_t *context, buffer_t *buf, size_t size) {

    size_t read_size;

    /* Limit size */
    read_size = (size > BCM283X_SPI_BUFFER_SIZE_MAX) ? BCM283X_SPI_BUFFER_SIZE_MAX : size;
    read_size = (read_size > context->receive_buffer.size) ? context->receive_buffer.size : read_size;

    buf->size = context->receive_buffer.size;
    strncpy(buf->data, context->receive_buffer.data, BCM283X_SPI_BUFFER_SIZE_MAX);
    context->receive_buffer.data[BCM283X_SPI_BUFFER_SIZE_MAX-1] = 0;
    /* Reset buffer size */
    context->receive_buffer.size = 0;

    /* Return read bytes */
    return read_size;

}

static ssize_t bcm283x_spi_rtdm_read_byte_rt(spi_bcm283x_context_t *context) {

    uint8_t value;
    /* Restore device spi settings */
    bcm2835_spi_setBitOrder(context->config.bit_order);
    bcm2835_spi_setDataMode(context->config.data_mode);
    bcm2835_spi_setClockDivider(context->config.clock_divider);
    bcm2835_spi_chipSelect(context->config.chip_select);
    bcm2835_spi_setChipSelectPolarity(context->config.chip_select, context->config.chip_select_polarity);
    CS_0();
    /* Return read bytes */
    value =  bcm2835_spi_transfer(0xFF);
    CS_1();
    return value;
}

/**
 * Read from the device.
 * @param[in] fd File descriptor.
 * @param[out] buf Input buffer as passed by the user.
 * @param[in] size Number of bytes the user requests to read.
 * @return On success, the number of bytes read. On failure return either -ENOSYS, to request that this handler be called again from the opposite realtime/non-realtime context, or another negative error code.
 */
static ssize_t bcm283x_spi_rtdm_read_rt(struct rtdm_fd *fd, void __user *buf, size_t size) {

	spi_bcm283x_context_t *context;
	ssize_t read_size;
    int res;
    /* Retrieve context */
    context = (spi_bcm283x_context_t *) rtdm_fd_to_private(fd);

	/* Limit size */
	read_size = (size > BCM283X_SPI_BUFFER_SIZE_MAX) ? BCM283X_SPI_BUFFER_SIZE_MAX : size;
	read_size = (read_size > context->receive_buffer.size) ? context->receive_buffer.size : read_size;



	/* Copy data to user space */
	res = rtdm_safe_copy_to_user(fd, buf, (const void *) context->receive_buffer.data, read_size);
	if (res) {
		printk(KERN_ERR "%s: Can't copy data from driver to user space (%d)!\r\n", __FUNCTION__, res);
		return (res < 0) ? res : -res;
	}

    return read_size;
}

/**
 * Write to the device.
 * @warning If unread data was present in the receive buffer, it will be overwritten.
 * @param[in] context The context associated with the device.
 * @param[in] buf Output buffer as passed by the user.
 * @param[in] size Number of bytes the user requests to write.
 * @return On success, the number of bytes written. On failure return either -ENOSYS, to request that this handler be called again from the opposite realtime/non-realtime context, or another negative error code.
 */
static ssize_t bcm283x_spi_rtdm_write_rt_using_context(spi_bcm283x_context_t *context, buffer_t *buf, size_t size) {
    int i = 0;
    size_t write_size;

    /* Ensure that there will be enough space in the buffer */
    if (size > BCM283X_SPI_BUFFER_SIZE_MAX) {
        printk(KERN_ERR "%s: Trying to transmit data larger than buffer size !", __FUNCTION__);
        return -EINVAL;
    }
    write_size = size;
    for(i =0; i < write_size; i++)
    context->transmit_buffer.data[i] = buf->data[i];

    context->transmit_buffer.size = write_size;

    /* Warn if receive buffer was not empty */
    if (context->receive_buffer.size > 0) {
        printk(KERN_WARNING "%s: Receive buffer was not empty and will be overwritten.\r\n", __FUNCTION__);
    }

    /* Restore device spi settings */
    bcm2835_spi_setBitOrder(context->config.bit_order);
    bcm2835_spi_setDataMode(context->config.data_mode);
    bcm2835_spi_setClockDivider(context->config.clock_divider);
    bcm2835_spi_chipSelect(context->config.chip_select);
    bcm2835_spi_setChipSelectPolarity(context->config.chip_select, context->config.chip_select_polarity);

    /* Initiate an outgoing transfer which will also store read content in input buffer. */
CS_0();
    bcm2835_spi_transfernb(context->transmit_buffer.data, context->receive_buffer.data, write_size,&(context->receive_buffer.size));
CS_1();

    /* Return bytes written */
    return write_size;

}

/**
 * Write to the device.
 * @warning If unread data was present in the receive buffer, it will be overwritten.
 * @param[in] context The context associated with the device.
 * @param[in] buf Output buffer as passed by the user.
 * @param[in] size Number of bytes the user requests to write.
 * @return On success, the number of bytes written. On failure return either -ENOSYS, to request that this handler be called again from the opposite realtime/non-realtime context, or another negative error code.
 */
static ssize_t bcm283x_spi_rtdm_write_byte_rt(spi_bcm283x_context_t *context, uint8_t byte) {

    context->transmit_buffer.size = 1;

    /* Warn if receive buffer was not empty */
    if (context->receive_buffer.size > 0) {
        printk(KERN_WARNING "%s: Receive buffer was not empty and will be overwritten.\r\n", __FUNCTION__);
    }

    /* Restore device spi settings */
    bcm2835_spi_setBitOrder(context->config.bit_order);
    bcm2835_spi_setDataMode(context->config.data_mode);
    bcm2835_spi_setClockDivider(context->config.clock_divider);
    bcm2835_spi_chipSelect(context->config.chip_select);
    bcm2835_spi_setChipSelectPolarity(context->config.chip_select, context->config.chip_select_polarity);

    /* Initiate an outgoing transfer which will also store read content in input buffer. */


    CS_0();

    bcm2835_spi_transfer(byte);
    CS_1();

    /* Return bytes written */
    return 1;

}

/**
 * Write to the device.
 * @warning If unread data was present in the receive buffer, it will be overwritten.
 * @param[in] fd File descriptor.
 * @param[in] buf Output buffer as passed by the user.
 * @param[in] size Number of bytes the user requests to write.
 * @return On success, the number of bytes written. On failure return either -ENOSYS, to request that this handler be called again from the opposite realtime/non-realtime context, or another negative error code.
 */
static ssize_t bcm283x_spi_rtdm_write_rt(struct rtdm_fd *fd, const void __user *buf, size_t size) {

	spi_bcm283x_context_t *context;
	buffer_t *temp_buffer;
	ssize_t  write_size;
	int res;

	/* Retrieve context */
	context = (spi_bcm283x_context_t *) rtdm_fd_to_private(fd);

	/* Save data in kernel space buffer */
	temp_buffer = (buffer_t *)vmalloc(sizeof(buffer_t));
	res = rtdm_safe_copy_from_user(fd, temp_buffer->data, buf, size);
	if (res) {
		printk(KERN_ERR "%s: Can't copy data from user space to driver (%d)!\r\n", __FUNCTION__, res);
		return (res < 0) ? res : -res;
	}
	write_size =  bcm283x_spi_rtdm_write_rt_using_context(context, temp_buffer, size);
	vfree(temp_buffer);
	return write_size;

}


/**
 * Changes the bit order setting for one device.
 * @param context The context associated with the device.
 * @param value An integer representing a bit order preference.
 * @return 0 on success, -EINVAL if the specified value is invalid.
 */
static int bcm283x_spi_change_bit_order(spi_bcm283x_context_t *context, const int value) {

	switch (value) {
		case BCM283X_SPI_BIT_ORDER_LSBFIRST:
		case BCM283X_SPI_BIT_ORDER_MSBFIRST:
			printk(KERN_DEBUG "%s: Changing bit order to %d.\r\n", __FUNCTION__, value);
			context->config.bit_order = value;
			return 0;
	}

	printk(KERN_ERR "%s: Unexpected value!\r\n", __FUNCTION__);
	return -EINVAL;

}

/**
 * Changes the data mode setting for one device.
 * @param context The context associated with the device.
 * @param value An integer representing a data mode.
 * @return 0 on success, -EINVAL if the specified value is invalid.
 */
static int bcm283x_spi_change_data_mode(spi_bcm283x_context_t *context, const int value) {

	switch (value) {
		case BCM283X_SPI_DATA_MODE_0:
		case BCM283X_SPI_DATA_MODE_1:
		case BCM283X_SPI_DATA_MODE_2:
		case BCM283X_SPI_DATA_MODE_3:
			printk(KERN_DEBUG "%s: Changing data mode to %d.\r\n", __FUNCTION__, value);
			context->config.data_mode = value;
			return 0;
	}

	printk(KERN_ERR "%s: Unexpected value!\r\n", __FUNCTION__);
	return -EINVAL;

}

/**
 * Changes the clock divider setting for one device.
 * @param context The context associated with the device.
 * @param value An integer representing a clock divider preference.
 * @return 0 on success, -EINVAL if the specified value is invalid.
 */
static int bcm283x_spi_change_clock_divider(spi_bcm283x_context_t *context, const int value) {

	switch (value) {
		case BCM283X_SPI_SPEED_4kHz:
		case BCM283X_SPI_SPEED_7kHz:
		case BCM283X_SPI_SPEED_15kHz:
		case BCM283X_SPI_SPEED_30kHz:
		case BCM283X_SPI_SPEED_61kHz:
		case BCM283X_SPI_SPEED_122kHz:
		case BCM283X_SPI_SPEED_244kHz:
		case BCM283X_SPI_SPEED_488kHz:
		case BCM283X_SPI_SPEED_976kHz:
		case BCM283X_SPI_SPEED_2MHz:
		case BCM283X_SPI_SPEED_4MHz:
		case BCM283X_SPI_SPEED_8MHz:
		case BCM283X_SPI_SPEED_15MHz:
		case BCM283X_SPI_SPEED_31MHz:
		case BCM283X_SPI_SPEED_62MHz:
		case BCM283X_SPI_SPEED_125MHz:
			printk(KERN_DEBUG "%s: Changing clock divider to %d.\r\n", __FUNCTION__, value);
			context->config.clock_divider = value;
			return 0;
	}

	printk(KERN_ERR "%s: Unexpected value!\r\n", __FUNCTION__);
	return -EINVAL;
}

/**
 * Changes the chip select polarity setting for one device.
 * @param context The context associated with the device.
 * @param value An integer representing a polarity preference.
 * @return 0 on success, -EINVAL if the specified value is invalid.
 */
static int bcm283x_spi_change_cs_polarity(spi_bcm283x_context_t *context, const int value) {

	switch (value) {
		case BCM283X_SPI_CS_POL_LOW:
		case BCM283X_SPI_CS_POL_HIGH:
			printk(KERN_DEBUG "%s: Changing chip select polarity to %d.\r\n", __FUNCTION__, value);
			context->config.chip_select_polarity = value;
			return 0;
	}

	printk(KERN_ERR "%s: Unexpected value!\r\n", __FUNCTION__);
	return -EINVAL;

}

/**
 * IOCTL handler.
 * @param[in] fd File descriptor.
 * @param[in] request Request number as passed by the user.
 * @param[in,out] arg Request argument as passed by the user.
 * @return A positive value or 0 on success. On failure return either -ENOSYS, to request that the function be called again from the opposite realtime/non-realtime context, or another negative error code.
 */
static int bcm283x_spi_rtdm_ioctl_rt(struct rtdm_fd *fd, unsigned int request, void __user *arg) {

	spi_bcm283x_context_t *context;
	int value;
	int res;

	/* Retrieve context */
	context = (spi_bcm283x_context_t *) rtdm_fd_to_private(fd);

	/*
	 * 	// Speed
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536);
	bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);
	 */


	/* Analyze request */
	switch (request) {

		case BCM283X_SPI_SET_BIT_ORDER: /* Change the bit order */
			res = rtdm_safe_copy_from_user(fd, &value, arg, sizeof(int));
			if (res) {
				printk(KERN_ERR "%s: Can't retrieve argument from user space (%d)!\r\n", __FUNCTION__, res);
				return (res < 0) ? res : -res;
			}
			return bcm283x_spi_change_bit_order(context, value);

		case BCM283X_SPI_SET_DATA_MODE: /* Change the data mode */
			res = rtdm_safe_copy_from_user(fd, &value, arg, sizeof(int));
			if (res) {
				printk(KERN_ERR "%s: Can't retrieve argument from user space (%d)!\r\n", __FUNCTION__, res);
				return (res < 0) ? res : -res;
			}
			return bcm283x_spi_change_data_mode(context, value);

		case BCM283X_SPI_SET_SPEED: /* Change the bus speed */
			res = rtdm_safe_copy_from_user(fd, &value, arg, sizeof(int));
			if (res) {
				printk(KERN_ERR "%s: Can't retrieve argument from user space (%d)!\r\n", __FUNCTION__, res);
				return (res < 0) ? res : -res;
			}
			return bcm283x_spi_change_clock_divider(context, value);

		case BCM283X_SPI_SET_CS_POLARITY: /* Change the chip select polarity */
			res = rtdm_safe_copy_from_user(fd, &value, arg, sizeof(int));
			if (res) {
				printk(KERN_ERR "%s: Can't retrieve argument from user space (%d)!\r\n", __FUNCTION__, res);
				return (res < 0) ? res : -res;
			}
			return bcm283x_spi_change_cs_polarity(context, value);

		default: /* Unexpected case */
			printk(KERN_ERR "%s: Unexpected request : %d!\r\n", __FUNCTION__, request);
			return -EINVAL;

	}

}

/**
 * This structure describes the RTDM driver.
 */
static struct rtdm_driver spi_bcm283x_driver = {
		.profile_info = RTDM_PROFILE_INFO(foo, RTDM_CLASS_EXPERIMENTAL, RTDM_SUBCLASS_GENERIC, 42),
		.device_flags = RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE | RTDM_FIXED_MINOR,
		.device_count = 2,
		.context_size = sizeof(struct spi_bcm283x_context_s),
		.ops = {
				.open = bcm283x_spi_rtdm_open,
				.read_rt = bcm283x_spi_rtdm_read_rt,
				.write_rt = bcm283x_spi_rtdm_write_rt,
				.ioctl_rt = bcm283x_spi_rtdm_ioctl_rt,
				.close = bcm283x_spi_rtdm_close
		}
};

/**
 * This function is called when the module is loaded. It initializes the
 * spi device using the bcm2835 libary, and registers the RTDM device.
 */
static int __init bcm283x_spi_rtdm_init(void) {

	int res;
	int device_id;

	/* Log */
	printk(KERN_INFO "%s: Starting driver ...", __FUNCTION__);

	/* Ensure cobalt is enabled */
	if (!realtime_core_enabled()) {
		printk(KERN_ERR "%s: Exiting as cobalt is not enabled!\r\n", __FUNCTION__);
		return -1;
	}

	/* Initialize the bcm2835 library */
	res = bcm2835_init();
	if (res != 1) {
		printk(KERN_ERR "%s: Error in bcm2835_init (%d).\r\n", __FUNCTION__, res);
		return -1;
	}


	/* Configure the spi port from bcm2835 library with arbitrary settings */
	bcm2835_spi_begin();
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
	bcm2835_spi_setDataMode(BCM2835_SPI_MODE1);
	bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_1024);
	bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);

	/* Prepare to register the two devices */
	for(device_id = 0; device_id < SPI_BCM283X_RTDM_DEVICES_NUMBER; device_id++){

        /*Set array open devices*/
        bcm283x_spi_rtdm_open_devices[device_id] = NULL;

		/* Set device parameters */
		spi_bcm283x_devices[device_id].driver = &spi_bcm283x_driver;
		spi_bcm283x_devices[device_id].label = "spidev0.%d";
		spi_bcm283x_devices[device_id].minor = device_id;

		/* Try to register the device */
		res = rtdm_dev_register(&spi_bcm283x_devices[device_id]);
		if (res == 0) {
			printk(KERN_INFO "%s: Device spidev0.%d registered without errors.\r\n", __FUNCTION__, device_id);
		} else {
			printk(KERN_ERR "%s: Device spidev0.%d registration failed : ", __FUNCTION__, device_id);
			switch (res) {
				case -EINVAL:
					printk(KERN_ERR "The descriptor contains invalid entries.\r\n");
					break;

				case -EEXIST:
					printk(KERN_ERR "The specified device name of protocol ID is already in use.\r\n");
					break;

				case -ENOMEM:
					printk(KERN_ERR "A memory allocation failed in the process of registering the device.\r\n");
					break;

				default:
					printk(KERN_ERR "Unknown error code returned.\r\n");
					break;
			}
			return res;
		}
	}

	return 0;

}

/**
 * This function is called when the module is unloaded. It unregisters the RTDM device.
 */
static void __exit bcm283x_spi_rtdm_exit(void) {

	int device_id;

	/* Log */
	printk(KERN_INFO "%s: Stopping driver ...\r\n", __FUNCTION__);

	/* Ensure cobalt is enabled */
	if (!realtime_core_enabled()) {
		printk(KERN_ERR "%s: Exiting as cobalt is not enabled!\r\n", __FUNCTION__);
		return;
	}

	/* Unregister the two devices */
	for (device_id = 0; device_id < SPI_BCM283X_RTDM_DEVICES_NUMBER; device_id++) {
        //if(bcm283x_spi_rtdm_open_devices[device_id] != NULL){
        //    bcm283x_spi_rtdm_open_devices[device_id]->device_used = 0;
        //}
		printk(KERN_INFO "%s: Unregistering device spidev0.%d  ...\r\n", __FUNCTION__, device_id);
		rtdm_dev_unregister(&spi_bcm283x_devices[device_id]);
		printk(KERN_INFO "%s: Device spidev0.%d unregistered  ...\r\n", __FUNCTION__, device_id);
	}

	/* Release the spi pins */
	bcm2835_spi_end();

	/* Unmap memory */
	bcm2835_close();

	/* Log */
	printk(KERN_INFO "%s: All done!\r\n", __FUNCTION__);

}

/*
 * Link init and exit functions with driver entry and exit points.
 */
module_init(bcm283x_spi_rtdm_init);
module_exit(bcm283x_spi_rtdm_exit);

/*
 * Register module values
 */
#ifndef GIT_VERSION
#define GIT_VERSION "0.1-untracked";
#endif /* ! GIT_VERSION */
MODULE_VERSION(GIT_VERSION);
MODULE_DESCRIPTION("Real-Time SPI driver for the Broadcom BCM283x SoC familly using the RTDM API");
MODULE_AUTHOR("Nicolas Schurando <schurann@ext.essilor.com>");
MODULE_AUTHOR("Piotr Piórkowski <qba100@gmail.com>");
MODULE_LICENSE("GPL v2");
EXPORT_SYMBOL(bcm283x_spi_rtdm_set_default_config);
EXPORT_SYMBOL(bcm283x_spi_rtdm_write_rt_using_context);
EXPORT_SYMBOL(bcm283x_spi_rtdm_read_rt_using_context);
EXPORT_SYMBOL(bcm283x_spi_change_bit_order);
EXPORT_SYMBOL(bcm283x_spi_change_data_mode);
EXPORT_SYMBOL(bcm283x_spi_change_clock_divider);
EXPORT_SYMBOL(bcm283x_spi_change_cs_polarity);
EXPORT_SYMBOL(bcm283x_spi_rtdm_read_byte_rt);
EXPORT_SYMBOL(bcm283x_spi_rtdm_write_byte_rt);
EXPORT_SYMBOL(bcm2835_gpio_write);
EXPORT_SYMBOL(bcm2835_gpio_lev);
EXPORT_SYMBOL(bcm2835_delayMicroseconds);
//EXPORT_SYMBOL(bcm283x_spi_rtdm_open_devices);