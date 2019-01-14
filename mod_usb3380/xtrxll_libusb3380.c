/*
 * xtrx low level usb3380 module source file
 * Copyright (c) 2017 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 * For more information, please visit: http://xtrx.io
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#include "xtrxll_port.h"

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <assert.h>

#include <libusb3380.h>

#include "xtrxll_log.h"
#include "xtrxll_base_pcie.h"
#include "xtrxll_libusb3380.h"

/* Include verilog file used for FPGA build */
#define localparam static const unsigned
#include "xtrxll_regs.vh"

#define ASYNC_MODE

#define BUFFERS_RX_MUL 2

enum {
	DMA_REGION_TX_ADDR = 0x40000000,  // 512Mb
	DMA_REGION_RX_ADDR = 0x20000000,  // 512Mb
};

enum msinterrupts {
	MSINT_SPI,
	MSINT_I2C,
	MSINT_PPS,
	MSINT_COUNT,
};
enum timeouts {
	TO_SPI = 50,
	TO_I2C = 50,
	TO_1PPS = 1500,

	TO_IRQ_POLL = 1250,
};

enum {
	// Not optimal, but use for now
    TXDMA_BUFFERS_AVAIL = (TXDMA_BUFFERS - 2) - 8,
	// Max performance (doesn't work well in low samplerate)
	//TXDMA_BUFFERS_AVAIL = TXDMA_BUFFERS,
};
#define MAX_EP_IN_FLY	2

#define DEV_NAME_SIZE       32
#define EXTRA_DATA_ALIGMENT 4096
struct xtrxll_usb3380_dev
{
	union {
		struct xtrxll_base_dev base;
		struct xtrxll_base_pcie_dma pcie;
	};
	unsigned tx_chans;
	uint32_t devid;
	bool rx_stop;
	bool rx_running;

	char pcie_devname[DEV_NAME_SIZE];

	libusb3380_context_t* ctx;
#ifdef ASYNC_MODE
	struct libusb3380_async_manager* mgr;
#endif

	uint32_t bar0;
	uint32_t bar1;

	pthread_mutex_t dev_mem_mutex;

	sem_t interrupts[MSINT_COUNT];

	// Input queue
	uint8_t* rx_queuebuf_ptr; // cache aligned pointer to rx_queuebuf
	uint8_t* tx_queuebuf_ptr; // cache aligned pointer to tx_queuebuf
	uint8_t* rx_discard_rxbuffer;

	//unsigned rx_bufsz;
	//unsigned tx_bufsz;
	unsigned rx_allocsz;
	unsigned tx_allocsz;

	//uint8_t queuebuf[BUFFERS_RX_MUL*2*RXDMA_MMAP_SIZE + TXDMA_MMAP_SIZE + EXTRA_DATA_ALIGMENT];
	//uint8_t discard_rxbuffer[2*RXDMA_MMAP_BUFF + 64];

	// RX part
	uint32_t rx_buf_max;
	uint32_t rx_buf_available; // number of buffers available for communication
	uint32_t rx_bufno_consumed;

	sem_t rx_buf_ready;
	sem_t rx_gpep_cleared;

	// TX part
	sem_t tx_buf_available;
	uint32_t tx_bufno_posted;

	// TODO: increase for more
	uint32_t tx_fly_buffers;

	unsigned tx_post_size[TXDMA_BUFFERS];

	unsigned tx_ep_count;
	unsigned rx_ep_count;
	unsigned rx_gpep_buffer_off[MAX_EP_IN_FLY];
	bool rx_gpep_active[MAX_EP_IN_FLY];

	bool rx_buf_to;
	bool rx_dma_flow_ctrl;

	bool tx_stop;
	bool rx_discard;
};


///////////////////////////////////////////////
// Library-wide variables

static char* strerror_safe(int err)
{
	//TODO check for MSVC!!!
#if defined(__linux) || defined(__APPLE__)
	static __thread char tmp_buffer[512];
	tmp_buffer[0] = 0;

	strerror_r(err, tmp_buffer, sizeof(tmp_buffer));
	return tmp_buffer;
#else
	return strerror(err);
#endif
}

static int pcieusb3380v0_reg_out(struct xtrxll_usb3380_dev* dev, unsigned reg,
								   uint32_t outval)
{
	uint32_t oval = htobe32(outval);
	int res = usb3380_async_pci_write32(dev->mgr, dev->bar0 + 4*reg,
										&oval, 1);
	XTRXLLS_LOG("USB3", XTRXLL_DEBUG_REGS, "%s: Write [%04x] = %08x (%d)\n",
			   dev->base.id, reg, outval, res);
	return res;
}

static int pcieusb3380v0_reg_in(struct xtrxll_usb3380_dev* dev, unsigned reg,
								uint32_t *pinval)
{
	uint32_t inval;
	int res;
	res = usb3380_async_pci_read32(dev->mgr, dev->bar0 + 4*reg, &inval, 1);
	inval = be32toh(inval);
	XTRXLLS_LOG("USB3", XTRXLL_DEBUG_REGS, "%s: Read  [%04x] = %08x (%d)\n",
			   dev->base.id, reg, inval, res);
	*pinval = inval;
	return res;
}

static int pcieusb3380v0_reg_out_n(struct xtrxll_usb3380_dev* dev,
								   unsigned streg, const uint32_t* outval,
								   unsigned count)
{
	int res;
	unsigned i;
	uint32_t to_write[count];
	for (i = 0; i < count; i++) {
		to_write[i] = htobe32(outval[i]);
	}
	res = usb3380_async_pci_write32(dev->mgr, dev->bar0 + 4*streg,
									to_write, count);

	XTRXLLS_LOG("USB3", XTRXLL_DEBUG_REGS, "%s: Write [%04x+%d] = %08x (%d)\n",
			   dev->base.id, streg, count, outval[0], res);
	return res;
}

static int pcieusb3380v0_reg_in_n(struct xtrxll_usb3380_dev* dev,
								  unsigned streg, uint32_t* inval,
								  unsigned count)
{
	unsigned i;
	uint32_t to_read[count];
	int res = usb3380_async_pci_read32(dev->mgr,dev->bar0 + 4*streg,
									   to_read, count);
	for (i = 0; i < count; i++) {
		inval[i] = be32toh(to_read[i]);
	}
	XTRXLLS_LOG("USB3", XTRXLL_DEBUG_REGS, "%s: Read [%04x+%d] = %08x (%d)\n",
			   dev->base.id, streg, count, inval[0], res);
	return res;
}


static int xtrxllusb3380v0_reg_out(struct xtrxll_base_dev* bdev, unsigned reg,
								   uint32_t outval)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	return pcieusb3380v0_reg_out(dev, reg, outval);
}

static int xtrxllusb3380v0_reg_in(struct xtrxll_base_dev* bdev, unsigned reg,
								  uint32_t *pinval)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	return pcieusb3380v0_reg_in(dev, reg, pinval);
}

static int xtrxllusb3380v0_reg_out_n(struct xtrxll_base_dev* bdev,
									 unsigned streg, const uint32_t* outval,
									 unsigned count)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	return pcieusb3380v0_reg_out_n(dev, streg, outval, count);
}

static int xtrxllusb3380v0_reg_in_n(struct xtrxll_base_dev* bdev,
									unsigned streg, uint32_t* inval,
									unsigned count)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	return pcieusb3380v0_reg_in_n(dev, streg, inval, count);
}

static int internal_xtrxll_transact_spi_wr(struct xtrxll_usb3380_dev* dev,
										   uint32_t value)
{
	return pcieusb3380v0_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_SPI_LMS7_0,
								   value);
}

static int internal_xtrxll_transact_spi_rb(struct xtrxll_usb3380_dev* dev,
										   uint32_t* pinval)
{
	return pcieusb3380v0_reg_in(dev, UL_GP_ADDR + GP_PORT_RD_SPI_LMS7_0,
								  pinval);
}

static int xtrxllusb3380_wait_msi(struct xtrxll_usb3380_dev* dev, enum msinterrupts i, int timeout_ms)
{
	int res;
	struct timespec ts;
	res = clock_gettime(CLOCK_REALTIME, &ts);
	if (res)
		return -EFAULT;

	if (timeout_ms > 0) {
		ts.tv_nsec += timeout_ms * 1000 * 1000;
		while (ts.tv_nsec > 1000 * 1000 * 1000) {
			ts.tv_nsec -= 1000 * 1000 * 1000;
			ts.tv_sec++;
		}
		res = sem_timedwait(&dev->interrupts[i], &ts);
	} else if (timeout_ms < 0) {
		res = sem_wait(&dev->interrupts[i]);
	} else {
		res = sem_trywait(&dev->interrupts[i]);
	}
	if (res) {
		// sem_* function on error returns -1, get proper error
		res = -errno;
	}
	return res;
}

static int xtrxllusb3380v0_lms7_spi_bulk(struct xtrxll_base_dev* bdev,
										 uint32_t lmsno, const uint32_t* out,
										 uint32_t* in, size_t count)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;

	if ((lmsno & XTRXLL_LMS7_0) == 0) {
		return -EINVAL;
	}

	int res;
	unsigned i;
	for (i = 0; i < count; ++i) {
		res = internal_xtrxll_transact_spi_wr(dev, out[i]);
		if (res) {
			return res;
		}

		res = xtrxllusb3380_wait_msi(dev, MSINT_SPI, TO_SPI);
		if (res) {
			XTRXLLS_LOG("USB3", XTRXLL_ERROR, "%s: SPI MSI wait timed out!\n",
						dev->base.id);
			return res;
		}

		if ((out[i] & (1U<<31)) == 0) {
			res = internal_xtrxll_transact_spi_rb(dev, &in[i]);
			if (res)
				return res;
		} else {
			in[i] = 0;
		}
		XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "%s: SPI[%d/%d] %08x => %08x\n",
				   dev->base.id, i, (unsigned)count, out[i], in[i]);
	}
	return 0;
}

static int xtrxllusb3380v0_i2c_cmd(struct xtrxll_base_dev* bdev,
								   uint32_t cmd, uint32_t *out)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;

	int res = pcieusb3380v0_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_TMP102, cmd);
	if (res)
		return res;

	if (out) {
		res = xtrxllusb3380_wait_msi(dev, MSINT_I2C, TO_I2C);
		if (res) {
			XTRXLLS_LOG("USB3", XTRXLL_ERROR, "%s: I2C MSI wait timed out!\n",
						dev->base.id);
			return res;
		}

		res = pcieusb3380v0_reg_in(dev, UL_GP_ADDR + GP_PORT_RD_TMP102, out);
	}
	return res;
}

static void xtrxllusb3380v0_on_msi_cb(void* param, int msinum, bool timedout)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)param;
	if (msinum < 0 && !timedout)
		return;

	XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "%s: MSI %d\n", dev->base.id, msinum);
	if (msinum == INT_RFIC0_SPI) {
		sem_post(&dev->interrupts[MSINT_SPI]);
	} else if (msinum == INT_I2C) {
		sem_post(&dev->interrupts[MSINT_I2C]);
	} else if (msinum == INT_1PPS) {
		sem_post(&dev->interrupts[MSINT_PPS]);
	}

	usb3380_msi_in_post(dev->mgr, TO_IRQ_POLL, xtrxllusb3380v0_on_msi_cb, dev);
}

static void xtrxllusb3380v0_log(libusb3380_loglevel_t level,
								void* obj,
								const char* func,
								const char* file,
								int line,
								const char* message, ...)
{
	(void)obj;

	enum xtrxll_loglevel log_lvlmap = XTRXLL_NONE;
	switch (level) {
	case USB3380_LOG_ERROR:   log_lvlmap = XTRXLL_ERROR; break;
	case USB3380_LOG_WARNING: log_lvlmap = XTRXLL_WARNING; break;
	case USB3380_LOG_INFO:    log_lvlmap = XTRXLL_INFO; break;
	case USB3380_LOG_DEBUG:   log_lvlmap = XTRXLL_DEBUG; break;
	case USB3380_LOG_DUMP:    log_lvlmap = XTRXLL_PARANOIC; break;
	}

	if (xtrxll_get_loglevel() < log_lvlmap)
		return;

	va_list ap;
	va_start(ap, message);
	xtrxll_vlog(log_lvlmap, "3380", func, file, line, message, ap);
	va_end(ap);
}

static libusb_device * find_usb_match(libusb_device **usbdev, size_t devices,
									  int usb_bus, int usb_port, int usb_addr,
									  unsigned* devid, char* devid_s, size_t devid_s_sz)
{
	struct libusb_device_descriptor desc;
	int res;
	for (unsigned i = 0; i < devices; i++) {
		res = libusb_get_device_descriptor(usbdev[i], &desc);
		if (res)
			continue;

		if (desc.idProduct != LIBUSB3380_PID || desc.idVendor != LIBUSB3380_VID)
			continue;

		uint8_t bus = libusb_get_bus_number(usbdev[i]);
		uint8_t port = libusb_get_port_number(usbdev[i]);
		uint8_t addr = libusb_get_device_address(usbdev[i]);

		*devid = (unsigned)bus * 1000000 + (unsigned)port * 1000 + addr;
		snprintf(devid_s, devid_s_sz, "%d/%d/%d", bus, port, addr);

		XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "usb3380: comparing devices %d/%d/%d <> %d/%d/%d\n",
					usb_bus, usb_port, usb_addr,
					bus, port, addr);

		if ((usb_addr == -1 && usb_bus == -1 && usb_port == -1) ||
				(usb_addr == -1 && usb_bus == -1 && usb_port == port) ||
				(usb_addr == -1 && usb_bus == bus && usb_port == port) ||
				(usb_bus == bus && usb_port == port && usb_addr == addr)) {
			return usbdev[i];
		}
	}
	return NULL;
}

static int xtrxllusb3380v0_open(const char* device, unsigned flags,
								struct xtrxll_base_dev** pdev)
{
	libusb3380_context_t* ctx;
	libusb3380_pcidev_t* pcidev;
	struct xtrxll_usb3380_dev* dev;

	int res;
	unsigned devid;
	char devid_s[16];

	int usb_bus = -1;
	int usb_port = -1;
	int usb_addr = -1;
	int usb_speed = 0;

	usb3380_set_logfunc(xtrxllusb3380v0_log, NULL);
	usb3380_set_loglevel(xtrxll_get_loglevel());

	if (device) {
		int cnt = sscanf(device, "usb3380://%d/%d/%d", &usb_bus, &usb_port, &usb_addr);
		if (cnt < 3) {
			cnt = sscanf(device, "usb3380://%d/%d", &usb_bus, &usb_port);
			if (cnt < 2) {
				cnt = sscanf(device, "usb3380://%d", &usb_bus);
				if (cnt < 1) {
					if (strcmp(device, "usb3380")) {
						XTRXLLS_LOG("USB3", XTRXLL_ERROR, "Can't parse device string!\n");
						return -EINVAL;
					}
				}
			}
		}
	}

	libusb_context *uctx;
	if (libusb_init(&uctx)) {
		return -EFAULT;
	}
	libusb_device **usbdev;
	ssize_t devices = libusb_get_device_list(uctx, &usbdev);
	if (devices < 0) {
		libusb_exit(uctx);
		return -ENODEV;
	}
	libusb_device *match = find_usb_match(usbdev, devices,
										  usb_bus, usb_port, usb_addr,
										  &devid, devid_s, sizeof(devid_s));
	if (match == NULL) {
		XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "No USB device was found to match %d/%d/%d\n",
					usb_bus, usb_port, usb_addr);
		libusb_exit(uctx);
		return -ENODEV;
	}

	int speed = libusb_get_device_speed(match);
	switch (speed) {
	case LIBUSB_SPEED_LOW: usb_speed = 1; break;
	case LIBUSB_SPEED_FULL: usb_speed = 12; break;
	case LIBUSB_SPEED_HIGH: usb_speed = 480; break;
	case LIBUSB_SPEED_SUPER: usb_speed = 5000; break;
	}
	bool int_polling = (speed < LIBUSB_SPEED_SUPER);

	res = usb3380_context_init_ex(&ctx, match, uctx);
	libusb_free_device_list(usbdev, 1);
	if (res) {
		XTRXLLS_LOG("USB3", XTRXLL_ERROR, "Unable to allocate USB3380 context: error: %d\n", res);
		libusb_exit(uctx);
		return res;
	}

	bool dual_ep = true;
	const char* env_dual_ep = getenv("XTRX_USB3380_DUAL_GPEP");
	if (env_dual_ep) {
		dual_ep = (atoi(env_dual_ep) > 0) ? true : false;
	}
	if (device && strstr(device, "nodualgpep") != NULL) {
		dual_ep = false;
	} else if (device && strstr(device, "dualgpep") != NULL) {
		dual_ep = true;
	}

	bool dual_ep_tx = true;
	const char* env_tx_dual_ep = getenv("XTRX_USB3380_TXDUAL_GPEP");
	if (env_tx_dual_ep) {
		dual_ep_tx = (atoi(env_tx_dual_ep) > 0) ? true : false;
	}
	if (device && strstr(device, "notxdualgpep") != NULL) {
		dual_ep_tx = false;
	} else if (device && strstr(device, "txdualgpep") != NULL) {
		dual_ep_tx = true;
	}

	XTRXLLS_LOG("USB3", XTRXLL_INFO, "USB3380 dual fly GPEP RX mode is %s, TX mode is %s\n",
			   (dual_ep) ? "on" : "off", (dual_ep_tx) ? "on" : "off");

	libusb3380_pcie_rc_cfg_t cfg;
	// USB OUT -> PCIe TX
	cfg.bar2.addr = DMA_REGION_TX_ADDR;
	cfg.bar2.length = BAR_1M;
	for (unsigned i = 0; i < 4; i++) {
		cfg.bar2.qadrants_ep_map[i] = (i < 2 && dual_ep_tx) ? LIBUSB3380_GPEP0 : LIBUSB3380_GPEP2;
		cfg.bar2.gpep_in_type[i] = false;
	}
	cfg.bar2.flags = 0;
	// USB IN <- PCIe RX
	cfg.bar3.addr = DMA_REGION_RX_ADDR;
	cfg.bar3.length = BAR_512M;
	for (unsigned i = 0; i < 4; i++) {
		cfg.bar3.qadrants_ep_map[i] = (i >= 2 && dual_ep) ? LIBUSB3380_GPEP2 : LIBUSB3380_GPEP0;
		cfg.bar3.gpep_in_type[i] = true;
	}
	cfg.bar3.flags = 0;
	cfg.gpep_fifo_in_size[0] = 4096;
	cfg.gpep_fifo_in_size[1] = 0;
	cfg.gpep_fifo_in_size[2] = (dual_ep) ? 4096 : 0;
	cfg.gpep_fifo_in_size[3] = 0;

	cfg.gpep_fifo_out_size[0] = (dual_ep_tx) ? 4096 : 0;
	cfg.gpep_fifo_out_size[1] = 0;
	cfg.gpep_fifo_out_size[2] = 4096;
	cfg.gpep_fifo_out_size[3] = 0;

	cfg.flags = 0;

	res = usb3380_init_root_complex(ctx, &cfg);
	if (res) {
		if (res == -EAGAIN && int_polling) {
			// Hack
			libusb_reset_device(*(libusb_device_handle**)ctx);

			usb3380_context_free(ctx);

			// Reinitialization takes time, wait 0.5sec
			usleep(500000);

			// Repeat reinitialization with the same usb_bus and usb_port,
			// addr will differ after usb reset
			if (libusb_init(&uctx)) {
				return -EFAULT;
			}
			devices = libusb_get_device_list(uctx, &usbdev);
			if (devices < 0) {
				libusb_exit(uctx);
				return -ENODEV;
			}
			match = find_usb_match(usbdev, devices, usb_bus, usb_port, -1,
								   &devid, devid_s, sizeof(devid_s));
			if (match == NULL) {
				libusb_exit(uctx);
				return -ENODEV;
			}

			res = usb3380_context_init_ex(&ctx, match, uctx);
			libusb_free_device_list(usbdev, 1);
			if (res) {
				XTRXLLS_LOG("USB3", XTRXLL_ERROR,
							"Unable to reinitialize context: error: %d\n", res);
				return res;
			}

			res = usb3380_init_root_complex(ctx, &cfg);
			if (res) {
				XTRXLLS_LOG("USB3", XTRXLL_ERROR,
							"Unable to intialize USB3380 Root Complex mode: error: %d\n", res);
				goto usbinit_fail;
			}
		}
	}

	res = usb3380_init_first_dev(ctx, 0, &pcidev);
	if (res) {
		if (res) {
			XTRXLLS_LOG("USB3", XTRXLL_ERROR, "No devices were found: error: %d\n", res);
			goto usbinit_fail;
		}
	}

	if (usb3380_pci_dev_did(pcidev) != XTRX_DID_V0 ||
			usb3380_pci_dev_vid(pcidev) != XTRX_VID_V0) {
		XTRXLLS_LOG("USB3", XTRXLL_ERROR, "Enumeared device isn't XTRX [%04x:%04x]\n",
				   usb3380_pci_dev_vid(pcidev),
				   usb3380_pci_dev_did(pcidev));
		goto usbinit_fail;
	}

	dev = (struct xtrxll_usb3380_dev*)malloc(sizeof(struct xtrxll_usb3380_dev));
	if (dev == NULL) {
		res = errno;
		XTRXLLS_LOG("USB3", XTRXLL_ERROR, "Can't allocate memory for device `%s`: %s\n",
				   device, strerror_safe(res));
		goto failed_malloc;
	}
	dev->ctx = ctx;
	dev->devid = devid;
	snprintf(dev->pcie_devname, DEV_NAME_SIZE - 1, "USB3_%s", devid_s);
	dev->bar0 = usb3380_pci_dev_bar_addr(pcidev, 0);
	dev->bar1 = usb3380_pci_dev_bar_addr(pcidev, 1);
	dev->rx_dma_flow_ctrl = true;

	res = pthread_mutex_init(&dev->dev_mem_mutex, NULL);
	if (res) {
		XTRXLLS_LOG("USB3", XTRXLL_ERROR, "%s: Failed to init mem mutex\n",
				   dev->base.id);
		goto failed_mutex_ctrl;
	}
	dev->rx_queuebuf_ptr = NULL;// (uint8_t*)((uintptr_t)(dev->queuebuf + EXTRA_DATA_ALIGMENT - 1) & ~((uintptr_t)(EXTRA_DATA_ALIGMENT - 1)));
	dev->tx_queuebuf_ptr = NULL; //dev->rx_queuebuf_ptr + BUFFERS_RX_MUL*2*RXDMA_MMAP_SIZE;
	dev->rx_discard_rxbuffer = NULL;

	dev->rx_allocsz = 0;
	dev->tx_allocsz = 0;

	dev->rx_running = false;
	dev->rx_stop = false;
	dev->rx_discard = false;
	res = xtrxllpciebase_init(&dev->pcie);
	if (res) {
		XTRXLLS_LOG("USB3", XTRXLL_ERROR, "%s: Failed to init DMA subsystem\n",
				   dev->base.id);
		goto failed_abi_ctrl;
	}
	*pdev = &dev->base;

	{
	libusb3380_configuration_t ucfg = {
		{ 1, 0, 1, 0 },
		{ TXDMA_BUFFERS, 0, TXDMA_BUFFERS, 0 },
	};
	res = usb3380_async_start(pcidev, &ucfg, &dev->mgr);
	}
	if (res) {
		XTRXLLS_LOG("USB3", XTRXLL_ERROR, "%s: unable to start USB3380 manager: %d",
				   dev->base.id, res);
		goto failed_async_start;
	}

	res = usb3380_async_set_gpep_timeout(dev->mgr, true, LIBUSB3380_GPEP0, 0, 250);
	if (res)
		goto failed_set_to;

	res = usb3380_async_set_gpep_timeout(dev->mgr, true, LIBUSB3380_GPEP2, 0, 250);
	if (res)
		goto failed_set_to;

	for (unsigned k = 0; k < TXDMA_BUFFERS; k++) {
		res = usb3380_async_set_gpep_timeout(dev->mgr, false, LIBUSB3380_GPEP0, k, 8000);
		if (res)
			goto failed_set_to;

		res = usb3380_async_set_gpep_timeout(dev->mgr, false, LIBUSB3380_GPEP2, k, 8000);
		if (res)
			goto failed_set_to;
	}

	dev->rx_buf_available = BUFFERS_RX_MUL * RXDMA_BUFFERS;
	dev->rx_buf_max = dev->rx_buf_available;
	dev->rx_bufno_consumed = 0;
	res = sem_init(&dev->rx_buf_ready, 0, 0);
	if (res)
		goto failed_sem_buf_ready;

	dev->rx_buf_to = false;

	res = sem_init(&dev->rx_gpep_cleared, 0, 0);
	if (res)
		goto failed_sem_gpep_cleared;

	dev->rx_ep_count = (dual_ep) ? 2 : 1;
	dev->rx_gpep_buffer_off[0] = 0;
	dev->rx_gpep_buffer_off[1] = 0;
	dev->rx_gpep_active[0] = false;
	dev->rx_gpep_active[1] = false;

	dev->tx_bufno_posted = 0;
	res = sem_init(&dev->tx_buf_available, 0, TXDMA_BUFFERS_AVAIL);
	if (res)
		goto failed_sem_tx_buf_available;

	dev->tx_ep_count = (dual_ep_tx) ? 2 : 1;
	dev->tx_fly_buffers = 0;
	dev->tx_stop = false;

	res = xtrxll_base_dev_init(&dev->base, xtrxllusb3380v0_init(XTRXLL_ABI_VERSION), dev->pcie_devname);
	if (res) {
		goto failed_unsup_hw;
	}

	res = pcieusb3380v0_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_INT_PCIE,
								(1U << INT_PCIE_E_FLAG) |
								((dev->rx_dma_flow_ctrl ? 0 : 1U) << 24) | /* disable ovf ctrl on RX path */
								(1 << INT_PCIE_I_FLAG) | (1 << INT_1PPS) |
								(1 << INT_RFIC0_SPI) | (1 << INT_I2C));
	if (res) {
		goto failed_pcie_cfg;
	}

	uint32_t dummy;
	res = pcieusb3380v0_reg_in(dev, UL_GP_ADDR + GP_PORT_RD_INTERRUPTS, &dummy);
	if (res) {
		goto failed_pcie_cfg;
	}

	res = xtrxllpciebase_dma_start(&dev->pcie, 0, NULL);
	if (res) {
		goto failed_pcie_cfg;
	}

	for (unsigned i = 0; i < MSINT_COUNT; i++) {
		res = sem_init(&dev->interrupts[i], 0, 0);
		if (res)
			goto failed_pcie_cfg;
	}

	res = usb3380_msi_in_post(dev->mgr, TO_IRQ_POLL, xtrxllusb3380v0_on_msi_cb, dev);
	if (res) {
		goto failed_pcie_cfg;
	}

	XTRXLLS_LOG("USB3", XTRXLL_INFO,  "%s: Device `%s` was opened (%d Mbit)\n",
			   dev->base.id, devid_s, usb_speed);
	return 0;

failed_unsup_hw:
failed_pcie_cfg:
	sem_destroy(&dev->tx_buf_available);
failed_sem_tx_buf_available:
	sem_destroy(&dev->rx_gpep_cleared);
failed_sem_gpep_cleared:
	sem_destroy(&dev->rx_buf_ready);
failed_sem_buf_ready:
failed_set_to:
	usb3380_async_stop(dev->mgr);
failed_async_start:
failed_abi_ctrl:
	pthread_mutex_destroy(&dev->dev_mem_mutex);
failed_mutex_ctrl:
	free(dev);
failed_malloc:
usbinit_fail:
	usb3380_context_free(ctx);
	return res;
}

static void xtrxllusb3380v0_close(struct xtrxll_base_dev* bdev)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;

	XTRXLLS_LOG("USB3", XTRXLL_INFO, "%s: Device closing\n", dev->base.id);

	usb3380_msi_in_cancel(dev->mgr);

	sem_destroy(&dev->tx_buf_available);
	sem_destroy(&dev->rx_gpep_cleared);
	sem_destroy(&dev->rx_buf_ready);

	usb3380_async_stop(dev->mgr);

	pthread_mutex_destroy(&dev->dev_mem_mutex);
	usb3380_context_free(dev->ctx);


	for (unsigned i = 0; i < MSINT_COUNT; i++) {
		sem_destroy(&dev->interrupts[i]);
	}
	free(dev->rx_queuebuf_ptr);
	free(dev->tx_queuebuf_ptr);
	free(dev);
}

static int xtrxllusb3380v0_discovery(xtrxll_device_info_t *buffer, size_t maxbuf)
{
	libusb_context* ctx;
	libusb_device **dev;
	int res = libusb_init(&ctx);
	if (res) {
		XTRXLLS_LOG("USB3", XTRXLL_ERROR, "Unable to initialize USB context\n");
		return res;
	}

	size_t found = 0;
	struct libusb_device_descriptor desc;
	ssize_t devices = libusb_get_device_list(ctx, &dev);
	if (devices < 0) {
		XTRXLLS_LOG("USB3", XTRXLL_ERROR, "Unable to list USB device\n");
		res = -EINTR;
		goto failed_get_list;
	}

	for (int i = 0; i < devices && found < maxbuf; i++) {
		res = libusb_get_device_descriptor(dev[i], &desc);
		if (res)
			continue;

		if (desc.idProduct != LIBUSB3380_PID || desc.idVendor != LIBUSB3380_VID)
			continue;

		uint8_t bus = libusb_get_bus_number(dev[i]);
		uint8_t port = libusb_get_port_number(dev[i]);
		uint8_t addr = libusb_get_device_address(dev[i]);

		snprintf(buffer[found].uniqname, sizeof(buffer[found].uniqname),
				 "usb3380://%d/%d/%d", bus, port, addr);

		int speed = libusb_get_device_speed(dev[i]);
		const char* speed_val = (speed == LIBUSB_SPEED_LOW) ? "1.5Mbit" :
						(speed == LIBUSB_SPEED_FULL) ? "12Mbit" :
						(speed == LIBUSB_SPEED_HIGH) ? "480Mbit" :
						(speed == LIBUSB_SPEED_SUPER) ? "5Gbit" : "<unknown>";
		snprintf(buffer[found].proto, sizeof(buffer[found].proto),
				 "usb3380 %s", speed_val);

		snprintf(buffer[found].addr, sizeof(buffer[found].addr),
				 "%d.%d:%d", bus, port, addr);

		strncpy(buffer[found].proto, "USB", sizeof(buffer[found].proto));
		strncpy(buffer[found].busspeed, speed_val, sizeof(buffer[found].busspeed));

		buffer[found].product_id = PRODUCT_XTRX;
		buffer[found].revision = 3; //TODO

		XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "usb3380: Found `%s` speed %s\n",
				   buffer[found].uniqname, speed_val);

		found++;
	}
	libusb_free_device_list(dev, 1);
	res = found;

failed_get_list:
	libusb_exit(ctx);
	return res;
}

static int xtrxllusb3380v0_dma_rx_init(struct xtrxll_base_dev* bdev, int chan,
									   unsigned buf_szs, unsigned* out_szs)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;

	int res;
	unsigned i;
	if (chan != 0)
		return -EINVAL;

	int allocsz = xtrxllpciebase_dmarx_bufsz(&dev->pcie, buf_szs);
	if (allocsz < 0)
		return allocsz;

	if (dev->rx_allocsz < allocsz) {
		free(dev->rx_queuebuf_ptr);
		dev->rx_allocsz = allocsz;
		res = posix_memalign((void**)&dev->rx_queuebuf_ptr, 4096, allocsz * (dev->rx_buf_max + 1));
		if (res != 0)
			return -res;

		dev->rx_discard_rxbuffer = dev->rx_queuebuf_ptr + dev->rx_buf_max * dev->rx_allocsz;
	}
	if (buf_szs == 0) {
		buf_szs = allocsz;
	}

	for (i = 0; i < RXDMA_BUFFERS; i++) {
		uint32_t off = DMA_REGION_RX_ADDR + ((i % 2) ? 256 * 1024 * 1024 : 0);
		uint32_t reg = (((buf_szs / 16) - 1) & 0xFFF) |
				(0xFFFFF000 & (off + (i / 2) * dev->rx_allocsz));

		res = pcieusb3380v0_reg_out(dev, UL_RXDMA_ADDR + i, reg);
		if (res)
			return res;
	}

	if ((GET_HWID_COMPAT(dev->base.hwid) == 1)) {
		uint32_t pktsz = (unsigned)((buf_szs / 16) - 1) | (1U << 31);

		res = pcieusb3380v0_reg_out(dev, UL_RXDMA_ADDR + 32, pktsz);
		if (res)
			return res;
	}

	dev->pcie.cfg_rx_bufsize = buf_szs;
	*out_szs = dev->pcie.cfg_rx_bufsize;
	return 0;
}

static int xtrxllusb3380v0_dma_rx_deinit(struct xtrxll_base_dev* bdev, int chan)
{
	if (chan != 0)
		return -EINVAL;
	return 0;
}

static int xtrxllusb3380v0_dma_rx_resume_at(struct xtrxll_base_dev *bdev,
											int chan, wts_long_t nxt)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	return xtrxllpciebase_dmarx_resume(&dev->pcie, chan, nxt);
}

static int xtrxllusb3380v0_dma_rx_gpep_issue(struct xtrxll_usb3380_dev* dev, unsigned ep_mask);

static void xtrxllusb3380v0_dma_rx_gpep_cb(const struct libusb3380_qgpep* gpep,
										   unsigned gpepno,
										   unsigned idx)
{
	unsigned ep_idx = (gpepno == LIBUSB3380_GPEP0) ? 0 : 1;
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)gpep->param;
	int res;
	bool packet_discarded = dev->rx_discard;

	if (gpep->base.status != DQS_SUCCESS) {
		if (dev->rx_stop) {
			dev->rx_gpep_active[ep_idx] = false;

			for (unsigned i = 0; i < dev->rx_ep_count; i++) {
				if (dev->rx_gpep_active[i])
					return;
			}

			sem_post(&dev->rx_gpep_cleared);
			return;
		}

		XTRXLLS_LOG("USB3", XTRXLL_WARNING, "%s: GPEP idx %d status: %d (witten %d)\n",
				   dev->base.id, ep_idx, gpep->base.status, gpep->base.written);

		dev->rx_gpep_buffer_off[ep_idx] += gpep->base.written;

		res = xtrxllusb3380v0_dma_rx_gpep_issue(dev, 1U << ep_idx);
		assert(res == 0); // TODO check error code
		return;
	}

	if (packet_discarded) {
		XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "%s: DISGARDED PACKET\n", dev->base.id);
	}

	// issue next URB
	if (!packet_discarded && ep_idx == 0) {
		dev->pcie.rd_buf_idx += dev->rx_ep_count;
	}
	dev->rx_gpep_buffer_off[ep_idx] = 0;

	if (!dev->rx_stop) {
		// If there's at least one available buffer, decrement availablity
		// counter and issue next gpep read commad
		unsigned available;
		do {
			available = __atomic_load_n(&dev->rx_buf_available, __ATOMIC_SEQ_CST);
			if (available == 0 /*|| available == 1 && ep0*/) {
				available = 0;
				break;
			}
		} while (!__atomic_compare_exchange_n(&dev->rx_buf_available, &available,
											  available - 1, false,
											  __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST));

		if (available) {
			dev->rx_discard = false;
			res = xtrxllusb3380v0_dma_rx_gpep_issue(dev, 1U << ep_idx);
			assert(res == 0);

			XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "%s: ISSUE READ %d buffer (%d avail) [%d%d]\n", dev->base.id,
					   (dev->pcie.rd_buf_idx + ep_idx) & (dev->rx_buf_max - 1), available,
					   dev->rx_gpep_active[0], dev->rx_gpep_active[1]);
		} else {
			// fill discard buffer
			dev->rx_discard = true;
			res = xtrxllusb3380v0_dma_rx_gpep_issue(dev, 1U << ep_idx);
			assert(res == 0);

			XTRXLLS_LOG("USB3", XTRXLL_WARNING, "%s: ISSUE DISCARD %d buffer (0 avail)\n", dev->base.id,
					   (dev->pcie.rd_buf_idx + ep_idx) & (dev->rx_buf_max - 1));
		}
	} else {
		dev->rx_gpep_active[ep_idx] = false;
	}

	// make buffer visible to waiting thread
	if (!packet_discarded) {
		// TODO timing advance due to dropped packet in the ring
		res = sem_post(&dev->rx_buf_ready);
		assert(res == 0);
	}
}

static int xtrxllusb3380v0_dma_rx_gpep_issue(struct xtrxll_usb3380_dev* dev,
											 unsigned ep_mask)
{
	int res;
	uint8_t* buffer;
	const uint8_t gpep_map[] = { LIBUSB3380_GPEP0, LIBUSB3380_GPEP2 };

	for (unsigned i = 0; i < dev->rx_ep_count; i++) {
		assert(dev->pcie.cfg_rx_bufsize > dev->rx_gpep_buffer_off[i]);

		if (ep_mask & (1U << i)) {
			buffer = (dev->rx_discard) ? dev->rx_discard_rxbuffer :
										 dev->rx_queuebuf_ptr + ((dev->pcie.rd_buf_idx + i) & (dev->rx_buf_max-1)) * dev->rx_allocsz;
			dev->rx_gpep_active[i] = true;
			res = usb3380_async_gpep_in_post(dev->mgr,
											 gpep_map[i],
											 0,
											 buffer + dev->rx_gpep_buffer_off[i],
											 dev->pcie.cfg_rx_bufsize - dev->rx_gpep_buffer_off[i],
											 xtrxllusb3380v0_dma_rx_gpep_cb,
											 dev);
			if (res)
				return res;
		}
	}

	return 0;
}

// Callback from internal IO thread take care to data races

static int xtrxllusb3380v0_dma_rx_getnext(struct xtrxll_base_dev* bdev,
										  int chan, void** addr, wts_long_t *wts,
										  unsigned *sz, unsigned flags,
										  unsigned timeout_ms)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	int res;

	if (chan != 0)
		return -EINVAL;
	if (dev->rx_stop)
		return -EINTR;

	struct timespec ats;
	if (flags & XTRXLL_RX_REPORT_TIMEOUT) {
		clock_gettime(CLOCK_REALTIME, &ats);
	}
	unsigned cnt = 0;
	for (;;) {
		// consume buffer, TODO error checking
		struct timespec ts;
		if ((flags & XTRXLL_RX_REPORT_TIMEOUT) && cnt == 0) {
			ts = ats;
		} else {
			clock_gettime(CLOCK_REALTIME, &ts);
		}

		if (flags & XTRXLL_RX_REPORT_TIMEOUT) {
			int64_t ms_diff = (ts.tv_sec - ats.tv_sec) * 1000 + (ts.tv_nsec - ats.tv_nsec) / 1000000;
			if (ms_diff > timeout_ms)
				return -EAGAIN;
		}

		// TODO calculate this size based on pkt arraival time
		ts.tv_nsec += ((timeout_ms > 0 && timeout_ms < 50) ? timeout_ms : 50 ) * 1000 * 1000;
		while (ts.tv_nsec > 1000 * 1000 * 1000) {
			ts.tv_nsec -= 1000 * 1000 * 1000;
			ts.tv_sec++;
		}

		res = sem_timedwait(&dev->rx_buf_ready, &ts);
		if (res) {
			if (errno == EINTR)
				continue;

			if (errno != ETIMEDOUT)
				return -EIO;
			else
				cnt++;

			if (dev->rx_stop)
				return -EINTR;

			wts_long_t cwts;
			unsigned bn;
			res = xtrxllpciebase_dmarx_get(&dev->pcie, chan, &bn, &cwts, sz,
										   (dev->rx_dma_flow_ctrl) ? PCIEDMARX_NO_CNTR_UPD : PCIEDMARX_NO_CNTR_CHECK, 0);
			if (res == 0) {
				XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "%s: RX DATA BUT NOT BUF\n", dev->base.id);
				continue;
			} else if (res == -EOVERFLOW) {
				XTRXLLS_LOG("USB3", XTRXLL_WARNING, "%s: RX OVERFLOW\n", dev->base.id);
				cwts += dev->pcie.cfg_rx_bufsize;
				dev->pcie.rd_cur_sample = cwts;
				if (*wts) {
					*wts = cwts;
				}

				if (!(flags & XTRXLL_RX_NOSTALL)) {
					return -EOVERFLOW;
				}
				if (dev->rx_stop)
					return -EOVERFLOW;

				xtrxllpciebase_dmarx_resume(&dev->pcie, chan, cwts);
			} else if (res == -EAGAIN) {
				XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "%s: RX AGAIN\n", dev->base.id);
				//xtrxllpciebase_dmarx_stat(&dev->pcie);
				continue;
			} else {
				XTRXLLS_LOG("USB3", XTRXLL_ERROR, "%s: Got %d!\n",
						   dev->base.id, res);
				return res;
			}
		} else {
			break;
		}
	}

	unsigned bufno = (dev->rx_bufno_consumed++) & (dev->rx_buf_max - 1);

	*sz = dev->pcie.cfg_rx_bufsize;
	*addr = dev->rx_queuebuf_ptr + bufno * dev->rx_allocsz;

	if (wts) {
		*wts = dev->pcie.rd_cur_sample;
		dev->pcie.rd_cur_sample += dev->pcie.rd_block_samples;
	}

	if (dev->rx_dma_flow_ctrl) {
		//TODO: MOVE AWAY FROM HERE INTO USB_IO THREAD !!!
		pcieusb3380v0_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_RXDMA_CNFRM, 0);
	}
	return 0;
}

static int xtrxllusb3380v0_dma_rx_release(struct xtrxll_base_dev* bdev,
										  int chan, void* addr)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	int res = 0;

	if (chan != 0 || dev->pcie.cfg_rx_bufsize == 0 || dev->rx_allocsz == 0)
		return -EINVAL;

	unsigned bufno = ((uint8_t*)addr - dev->rx_queuebuf_ptr) / dev->rx_allocsz;
	unsigned bufoff = ((uint8_t*)addr - dev->rx_queuebuf_ptr) % dev->rx_allocsz;
	if (bufno >= dev->rx_buf_max || bufoff)
		return -EINVAL;

	unsigned buffs_available =  __atomic_add_fetch(&dev->rx_buf_available, 1, __ATOMIC_SEQ_CST);
	if (buffs_available == 1 && !dev->rx_stop) {
		// We've got transition from 0 -> 1, so start read process into this buffer
		dev->rx_discard = false;
		res = xtrxllusb3380v0_dma_rx_gpep_issue(dev, (1U << dev->rx_ep_count) - 1);
	}

	XTRXLLS_LOG("USB3", XTRXLL_DEBUG,  "%s: RX DMA RELEASE %d\n",
			   dev->base.id, bufno);
	return res;
}


// TX DMA

static int xtrxllusb3380v0_dma_tx_init(struct xtrxll_base_dev* bdev, int chan,
									   unsigned buf_szs)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	int res;

	if (chan != 0)
		return -EINVAL;

	unsigned allocsz = xtrxllpciebase_dmatx_bufsz(&dev->pcie, buf_szs);
	if (allocsz < 0)
		return allocsz;

	if (dev->tx_allocsz < allocsz) {
		free(dev->tx_queuebuf_ptr);
		dev->tx_allocsz = allocsz;
		res = posix_memalign((void**)&dev->tx_queuebuf_ptr, 4096, allocsz * TXDMA_BUFFERS);
		if (res != 0)
			return -res;
	}

	unsigned i;
	for (i = 0; i < TXDMA_BUFFERS; i++) {
		int num = (i % 2) ? TXDMA_BUFFERS / 2 + i / 2 : i / 2;

		uint32_t reg = (((buf_szs / 16) - 1) & 0xFFF) |
				(0xFFFFF000 & (DMA_REGION_TX_ADDR + /*i*/ num * TXDMA_MMAP_BUFF));

		XTRXLLS_LOG("USB3", XTRXLL_INFO,  "%s: TX buf %d  DMA ADDR 0x%08x\n",
				   dev->base.id, i, reg);
		res = pcieusb3380v0_reg_out(dev, UL_TXDMA_ADDR + i, reg);
		if (res)
			return res;
	}

	return 0;
}

static int xtrxllusb3380v0_dma_tx_deinit(struct xtrxll_base_dev* bdev, int chan)
{
	if (chan !=0)
		return -EINVAL;

	return 0;
}

static void xtrxllusb3380v0_dma_tx_gpepx_cb(const struct libusb3380_qgpep* gpep,
											unsigned gpepno,
											unsigned idx)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)gpep->param;
	int res;
	uint32_t fly = __atomic_and_fetch(&dev->tx_fly_buffers, ~(1U << idx), __ATOMIC_SEQ_CST);

	XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "%s: WRITE DONE %d idx %d (%08x)\n",
			   dev->base.id, gpepno, idx, fly);

	res = sem_post(&dev->tx_buf_available);
	assert(res == 0); 

	if (gpep->base.status != DQS_SUCCESS) {
		if (dev->tx_stop) {
			return;
		}

		XTRXLLS_LOG("USB3", XTRXLL_ERROR, "%s: GPEP_OUT %d idx %d status: %d\n",
				   dev->base.id, gpepno, idx, gpep->base.status);

		// TODO handle TX timeout properly
		// abort();
	}
}

static int xtrxllusb3380v0_dma_tx_gpep_issue(struct xtrxll_usb3380_dev* dev, unsigned idx)
{
	uint8_t* buffer = dev->tx_queuebuf_ptr + (idx & 0x1f) * TXDMA_MMAP_BUFF;
	unsigned length = dev->tx_post_size[idx & 0x1f];
	int res;

	XTRXLLS_LOG("USB3", XTRXLL_DEBUG, "%s: TX send idx %d\n", dev->base.id, idx);

	libusb3380_gpep_t gp = (dev->tx_ep_count == 1) ? LIBUSB3380_GPEP2 :
						(((idx % 2) == 0) ? LIBUSB3380_GPEP0 : LIBUSB3380_GPEP2);
	res = usb3380_async_gpep_out_post(dev->mgr,
								gp,
								idx & 0x1f,
								buffer,
								length,
								xtrxllusb3380v0_dma_tx_gpepx_cb,
								dev);
	return res;
}

static int xtrxllusb3380v0_dma_tx_getfree_ex(struct xtrxll_base_dev* bdev,
											 int chan, void** addr,
											 uint16_t* late, unsigned timeout_ms)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	int res;
	int ilate = 0;
	unsigned bufno = TXDMA_MMAP_BUFF; // should not happed

	if (chan != 0)
		return -EINVAL;

	if (addr == NULL) {
		res = xtrxllpciebase_dmatx_get(&dev->pcie, chan, NULL, &ilate, true);
		if (late) {
			*late = ilate;
		}
		return res;
	}

	bool bufno_obtained = false;
	bool got_buf = false;

	struct timespec ats;
	if (timeout_ms != ~0U) {
		clock_gettime(CLOCK_REALTIME, &ats);
	}


	for (; !bufno_obtained || !got_buf;) {
		struct timespec ts;
		if (timeout_ms == ~0U) {
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_nsec += 100 * 1000 * 1000;
		} else {
			ts = ats;
			ts.tv_nsec += timeout_ms * 1000UL * 1000UL;
		}

		while (ts.tv_nsec > 1000 * 1000 * 1000) {
			ts.tv_nsec -= 1000 * 1000 * 1000;
			ts.tv_sec++;
		}

		if (!got_buf) {
			res = sem_timedwait(&dev->tx_buf_available, &ts);
			if (res) {
				if (errno == EINTR) {
					continue;
				} else if (errno == ETIMEDOUT) {
					if (dev->tx_stop) {
						return -EIO;
					}
					if (timeout_ms == ~0U)
						continue;

					return res;
				} else {
					return -EIO;
				}
			} else {
				got_buf = true;
			}
		}

		if (dev->tx_stop) {
			return -EIO;
		}

		if (!bufno_obtained) {
			res = xtrxllpciebase_dmatx_get(&dev->pcie, chan, &bufno, &ilate, false);
			if (res == -EBUSY) {
				XTRXLLS_LOG("USB3", XTRXLL_WARNING,  "%s: TX BUSY\n", dev->base.id);

				xtrxllpciebase_dmatx_get(&dev->pcie, chan, NULL, &ilate, true);
			} else if (res == 0) {
				assert(bufno == (dev->tx_bufno_posted & 0x1f));
				dev->tx_bufno_posted++;

				bufno_obtained = true;
			} else {
				return res;
			}
		}
	}

	if (late) {
		*late = ilate;
	}
	if (addr) {
		*addr = (void*)((char*)dev->tx_queuebuf_ptr + TXDMA_MMAP_BUFF * bufno);
	}

	return 0;
}

static int xtrxllusb3380v0_dma_tx_post(struct xtrxll_base_dev* bdev, int chan,
									   void* addr, wts_long_t wts,
									   uint32_t samples)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	unsigned bufno = ((char*)addr - (char*)dev->tx_queuebuf_ptr) / TXDMA_MMAP_BUFF;

	assert(bufno < TXDMA_BUFFERS);
	int written = 0;
	int res;

	dev->tx_post_size[bufno] = samples * 8;
	__atomic_or_fetch(&dev->tx_fly_buffers, 1U << bufno, __ATOMIC_SEQ_CST);

	res = xtrxllusb3380v0_dma_tx_gpep_issue(dev, bufno);
	if (res) {
		unsigned fly = __atomic_and_fetch(&dev->tx_fly_buffers, ~(1U << bufno), __ATOMIC_SEQ_CST);
		XTRXLLS_LOG("USB3", XTRXLL_ERROR, "%s: POST ERR: %d (%08x)\n", dev->base.id, res, fly);
		return res;
	}

	res = xtrxllpciebase_dmatx_post(&dev->pcie, chan, bufno,
									wts, samples);
	if (res) {
		XTRXLLS_LOG("USB3", XTRXLL_ERROR,  "%s: TX POST failed buf %d: error %d (written %d)\n",
				   dev->base.id, bufno, res, written);
		return res;
	}

	//res = xtrxllusb3380v0_dma_tx_gpep_issue(dev, bufno, "XXX");
	return res;
}

static int xtrxllusb3380v0_repeat_tx_buf(struct xtrxll_base_dev* bdev, int chan,
										 xtrxll_fe_t fmt, const void* buff,
										 unsigned buf_szs, xtrxll_mode_t mode)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	int err;

	err = xtrxllpciebase_repeat_tx(&dev->pcie, chan, fmt, buf_szs, mode);
	if (err)
		return err;

	unsigned dw_cnt = buf_szs / 4;
	unsigned cmpltd;
	unsigned t;

	for (cmpltd = 0; cmpltd != dw_cnt;) {
		t = dw_cnt - cmpltd;
		if (t > 128 / 4)
			t = 128 / 4;

		err = usb3380_pci_dev_mem_write32_n(dev->ctx,
											dev->bar1 + 4 * cmpltd,
											((const uint32_t*)buff) + cmpltd,
											t);
		if (err)
			return err;

		cmpltd += t;
	}

	return 0;
}

static int xtrxllusb3380v0_repeat_tx_start(struct xtrxll_base_dev* bdev,
										   int chan, int start)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	return xtrxllpciebase_repeat_tx_start(&dev->pcie, chan, start);
}

static void xtrxllusb3380v0_flush_dma_tx(struct xtrxll_usb3380_dev* dev)
{
	int res;
	unsigned fly;
	for (uint32_t i = 1, idx = 0; i != 0; i <<= 1, idx++) {
		__atomic_load(&dev->tx_fly_buffers, &fly, __ATOMIC_SEQ_CST);
		if (fly & i) {
			libusb3380_gpep_t gp = (dev->tx_ep_count == 1) ? LIBUSB3380_GPEP2 :
								(((idx % 2) == 0) ? LIBUSB3380_GPEP0 : LIBUSB3380_GPEP2);
			res = usb3380_async_gpep_cancel(dev->mgr, false, gp, idx);
			if (res != 0) {
				XTRXLLS_LOG("USB3", XTRXLL_ERROR,  "%s: TX FLUSH DMA idx %d: error %d\n",
						   dev->base.id, idx, res);
			}
		}
	}
}

static int xtrxllusb3380v0_dma_start(struct xtrxll_base_dev* bdev, int chan,
									 const struct xtrxll_dmaop* op)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	dev->tx_chans = (op) ? ((op->txmode == XTRXLL_FE_MODE_MIMO) ? 2 : 1) : 2;

	int res;
	xtrxll_fe_t rxfe = (op) ? op->rxfe : XTRXLL_FE_STOP;
	xtrxll_fe_t txfe = (op) ? op->txfe : XTRXLL_FE_STOP;

	if (rxfe == XTRXLL_FE_STOP) {
		dev->rx_stop = true;
		xtrxllpciebase_dmarx_stat(&dev->pcie);

		res = pcieusb3380v0_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_RXTXDMA,
									(1UL << GP_PORT_WR_RXTXDMA_RXV) | ((unsigned)rxfe << GP_PORT_WR_RXTXDMA_RXOFF));

		while (dev->rx_gpep_active[0] || dev->rx_gpep_active[1]) {
			usleep(1000);
		}
	} else if (rxfe != XTRXLL_FE_DONTTOUCH) {
		dev->rx_stop = false;
		dev->rx_bufno_consumed = 0;
		dev->rx_buf_available = dev->rx_buf_max;
		sem_destroy(&dev->rx_buf_ready);
		sem_init(&dev->rx_buf_ready, 0, 0);
	}

	if (txfe == XTRXLL_FE_STOP) {
		dev->tx_stop = true;
		//xtrxllpciebase_dmatx_get(&dev->pcie, 0, NULL, NULL, false);
	} else if (txfe != XTRXLL_FE_DONTTOUCH) {
		dev->tx_stop = false;
		//dev->tx_buf_available = TXDMA_BUFFERS;
		//if (sem_getvalue(&dev->tx_buf_available) != TXDMA_BUFFERS)
		sem_destroy(&dev->tx_buf_available);
		sem_init(&dev->tx_buf_available, 0, TXDMA_BUFFERS_AVAIL);

		dev->tx_bufno_posted = 0;
	}


	res = xtrxllpciebase_dma_start(&dev->pcie, chan, op);
	if (rxfe == XTRXLL_FE_STOP) {
		while (dev->rx_gpep_active[0] || dev->rx_gpep_active[1]) {
			struct timespec ts;
			clock_gettime(CLOCK_REALTIME, &ts);

			ts.tv_nsec += 500 * 1000 * 1000;
			if (ts.tv_nsec > 1000 * 1000 * 1000) {
				ts.tv_nsec -= 1000 * 1000 * 1000;
				ts.tv_sec++;
			}

			sem_timedwait(&dev->rx_gpep_cleared, &ts);
		}
	} else if (rxfe != XTRXLL_FE_DONTTOUCH) {
		// TODO fixme!
		memset(dev->rx_gpep_buffer_off, 0, sizeof(dev->rx_gpep_buffer_off));
		xtrxllusb3380v0_dma_rx_gpep_issue(dev, (1U << dev->rx_ep_count) - 1);
	}

	if (txfe == XTRXLL_FE_STOP) {
		xtrxllusb3380v0_flush_dma_tx(dev);
		for (;;) {
			unsigned fly;
			__atomic_load(&dev->tx_fly_buffers, &fly, __ATOMIC_SEQ_CST);
			if (fly == 0)
				break;
			usleep(1000);
		}
	}

	return res;
}

static int xtrxllusb3380v0_get_sensor(struct xtrxll_base_dev* bdev,
									  unsigned sensorno, int* outval)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;

	switch (sensorno) {
	case XTRXLL_DMABUF_RXST64K:
		*outval = (dev->rx_buf_max - dev->rx_buf_available) * 65536 / dev->rx_buf_max;
		return 0;
	case XTRXLL_DMABUF_TXST64K:
		sem_getvalue(&dev->tx_buf_available, outval);
		*outval = (TXDMA_BUFFERS - *outval) * 65536 / TXDMA_BUFFERS;
		return 0;
	case XTRXLL_ONEPPS_CAPTURED:
		return xtrxllusb3380_wait_msi(dev, MSINT_PPS, TO_1PPS);
	default:
		return bdev->ctrlops->get_sensor(bdev->self, sensorno, outval);
	}
}

static int xtrxllusb3380v0_set_param(struct xtrxll_base_dev* dev,
									  unsigned paramno, uintptr_t val)
{
	return dev->ctrlops->set_param(dev->self, paramno, val);
}

static const char* get_proto_id(void) {
	return "usb3380";
}

const static struct xtrxll_ops mod_ops = {
	.open = xtrxllusb3380v0_open,
	.close = xtrxllusb3380v0_close,
	.discovery = xtrxllusb3380v0_discovery,
	.get_proto_id = get_proto_id,

	.reg_out = xtrxllusb3380v0_reg_out,
	.reg_in = xtrxllusb3380v0_reg_in,

	.reg_out_n = xtrxllusb3380v0_reg_out_n,
	.reg_in_n = xtrxllusb3380v0_reg_in_n,

	.spi_bulk = xtrxllusb3380v0_lms7_spi_bulk,
	.i2c_cmd = xtrxllusb3380v0_i2c_cmd,

	// RX DMA
	.dma_rx_init = xtrxllusb3380v0_dma_rx_init,
	.dma_rx_deinit = xtrxllusb3380v0_dma_rx_deinit,

	.dma_rx_getnext = xtrxllusb3380v0_dma_rx_getnext,
	.dma_rx_release = xtrxllusb3380v0_dma_rx_release,

	.dma_rx_resume_at = xtrxllusb3380v0_dma_rx_resume_at,

	// TX DMA
	.dma_tx_init = xtrxllusb3380v0_dma_tx_init,
	.dma_tx_deinit = xtrxllusb3380v0_dma_tx_deinit,
	.dma_tx_getfree_ex = xtrxllusb3380v0_dma_tx_getfree_ex,
	.dma_tx_post = xtrxllusb3380v0_dma_tx_post,
	.dma_start = xtrxllusb3380v0_dma_start,

	.repeat_tx_buf = xtrxllusb3380v0_repeat_tx_buf,
	.repeat_tx_start = xtrxllusb3380v0_repeat_tx_start,

	.get_sensor = xtrxllusb3380v0_get_sensor,
	.set_param = xtrxllusb3380v0_set_param,
};

const struct xtrxll_ops* xtrxllusb3380v0_init(unsigned abi_version)
{
	if (XTRXLL_ABI_VERSION == abi_version) {
		return &mod_ops;
	}

	return NULL;
}

#ifndef XTRXLL_STATIC
const struct xtrxll_ops* xtrxll_init(unsigned abi_version)
{
	return xtrxllusb3380v0_init(abi_version);
}
#endif

