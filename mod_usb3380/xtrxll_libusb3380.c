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
#include "xtrxll_base.h"
#include "xtrxll_base_pcie.h"
#include "xtrxll_libusb3380.h"

/* Include verilog file used for FPGA build */
#define localparam static const unsigned
#include "xtrxll_regs.vh"

#define ASYNC_MODE

enum {
	DMA_REGION_0_ADDR = 0x20000000,  // 256Mb
	DMA_REGION_1_ADDR = 0x10000000,  // 256Mb
};

#define DEV_NAME_SIZE       32
#define EXTRA_DATA_ALIGMENT 4096
struct xtrxll_usb3380_dev
{
	union {
		struct xtrxll_base_dev base;
		struct xtrxll_base_pcie_dma pcie;
	};
	unsigned tx_chans;
	uint16_t devid;
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

	// Input queue
	uint8_t* rx_queuebuf_ptr; // cache aligned pointer to rx_queuebuf
	uint8_t* tx_queuebuf_ptr; // cache aligned pointer to tx_queuebuf

	uint8_t queuebuf[2*RXDMA_MMAP_SIZE + TXDMA_MMAP_SIZE + EXTRA_DATA_ALIGMENT];

	// RX part
	uint32_t rx_buf_available; // number of buffers available for communication
	uint32_t rx_bufno_consumed;

	sem_t rx_buf_ready;
	sem_t rx_gpep_cleared;

	// TX part
	uint32_t tx_buf_ready; // number of buffers ready to post
	uint32_t tx_bufno_posted;

	sem_t tx_buf_available;

	uint32_t tx_bufno_consumed; //consumed by IO
	unsigned tx_post_size[TXDMA_BUFFERS];
	unsigned tx_bufs_in_transfer;

	bool rx_gpep_active;
	bool rx_buf_to;

	bool tx_gpep_active;

	bool rx_dma_flow_ctrl;

	bool tx_stop;
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

#ifndef ASYNC_MODE
static int pcieusb3380v0_reg_out(struct xtrxll_usb3380_dev* dev, unsigned reg,
								   uint32_t outval)
{
	int res;
	pthread_mutex_lock(&dev->dev_mem_mutex);
	res = usb3380_pci_dev_mem_write32(dev->ctx, dev->bar0 + 4*reg,
										  htobe32(outval));
	pthread_mutex_unlock(&dev->dev_mem_mutex);
	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Write [%04x] = %08x\n",
			   dev->base.id, reg, outval);

	switch (res) {
	case 0: return 0;
	case LIBUSB_ERROR_TIMEOUT: return -ETIMEDOUT;
	case LIBUSB_ERROR_IO: return -EIO;
	case LIBUSB_ERROR_NO_DEVICE: return -ENODEV;
	case LIBUSB_ERROR_NOT_FOUND: return -ENXIO;
	default: return -EFAULT;
	}
}

static int pcieusb3380v0_reg_in(struct xtrxll_usb3380_dev* dev, unsigned reg,
								uint32_t *pinval)
{
	uint32_t inval;
	int res;
	pthread_mutex_lock(&dev->dev_mem_mutex);
	res = usb3380_pci_dev_mem_read32(dev->ctx, dev->bar0 + 4*reg, &inval);
	pthread_mutex_unlock(&dev->dev_mem_mutex);

	switch (res) {
	case 0: break;
	case LIBUSB_ERROR_TIMEOUT: return -ETIMEDOUT;
	case LIBUSB_ERROR_IO: return -EIO;
	case LIBUSB_ERROR_NO_DEVICE: return -ENODEV;
	case LIBUSB_ERROR_NOT_FOUND: return -ENXIO;
	default: return -EFAULT;
	}

	inval = be32toh(inval);
	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Read  [%04x] = %08x\n",
			   dev->base.id, reg, inval);
	*pinval = inval;
	return 0;
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

	pthread_mutex_lock(&dev->dev_mem_mutex);
	res = usb3380_pci_dev_mem_write32_n(dev->ctx, dev->bar0 + 4*streg,
										to_write, count);
	pthread_mutex_unlock(&dev->dev_mem_mutex);

	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Write [%04x+%d] = %08x\n",
			   dev->base.id, streg, count, outval[0]);

	switch (res) {
	case 0: return 0;
	case LIBUSB_ERROR_TIMEOUT: return -ETIMEDOUT;
	case LIBUSB_ERROR_IO: return -EIO;
	case LIBUSB_ERROR_NO_DEVICE: return -ENODEV;
	case LIBUSB_ERROR_NOT_FOUND: return -ENXIO;
	default: return -EFAULT;
	}
}

static int pcieusb3380v0_reg_in_n(struct xtrxll_usb3380_dev* dev,
								  unsigned streg, uint32_t* inval,
								  unsigned count)
{
	unsigned i;
	uint32_t to_read[count];

	int res;
	pthread_mutex_lock(&dev->dev_mem_mutex);
	res = usb3380_pci_dev_mem_read32_n(dev->ctx, dev->bar0 + 4*streg,
									   to_read, count);
	pthread_mutex_unlock(&dev->dev_mem_mutex);

	switch (res) {
	case 0: break;
	case LIBUSB_ERROR_TIMEOUT: return -ETIMEDOUT;
	case LIBUSB_ERROR_IO: return -EIO;
	case LIBUSB_ERROR_NO_DEVICE: return -ENODEV;
	case LIBUSB_ERROR_NOT_FOUND: return -ENXIO;
	default: return -EFAULT;
	}

	for (i = 0; i < count; i++) {
		inval[i] = be32toh(to_read[i]);
	}

	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Read [%04x+%d] = %08x\n",
			   dev->base.id, streg, count, inval[0]);
	return 0;
}
#else


static int pcieusb3380v0_reg_out(struct xtrxll_usb3380_dev* dev, unsigned reg,
								   uint32_t outval)
{
	uint32_t oval = htobe32(outval);
	int res = usb3380_async_pci_write32(dev->mgr, dev->bar0 + 4*reg,
										&oval, 1);
	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Write [%04x] = %08x (%d)\n",
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
	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Read  [%04x] = %08x (%d)\n",
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

	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Write [%04x+%d] = %08x (%d)\n",
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
	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Read [%04x+%d] = %08x (%d)\n",
			   dev->base.id, streg, count, inval[0], res);
	return res;
}
#endif

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

#ifdef ASYNC_MODE
		res = usb3380_async_await_msi(dev->mgr, 0);
#else
		// TODO not working well!
		res = usb3380_pci_wait_interrupt(dev->ctx, 1250);
#endif
		if (res) {
			XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: SPI[%d/%d] INT err:%d!\n",
					   dev->base.id, i, (unsigned)count, res);
			return res;
		}

		uint32_t dummy, dummy2;
		res = pcieusb3380v0_reg_in(dev, UL_GP_ADDR + GP_PORT_RD_INTERRUPTS, &dummy);
		res = pcieusb3380v0_reg_in(dev, UL_GP_ADDR + GP_PORT_RD_INTERRUPTS, &dummy2);

		if ((out[i] & (1U<<31)) == 0) {
			res = internal_xtrxll_transact_spi_rb(dev, &in[i]);
			if (res)
				return res;
		} else {
			in[i] = 0;
		}
		XTRXLL_LOG(XTRXLL_DEBUG, "XTRX %s: SPI[%d/%d] %08x => %08x (%08x %08x)\n",
				   dev->base.id, i, (unsigned)count, out[i], in[i], dummy, dummy2);
	}
	return 0;
}

static void xtrxllusb3380v0_log(libusb3380_loglevel_t level,
								void* obj, const char* message, ...)
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

	char tmp_buf[1024];

	va_list ap;
	va_start(ap, message);
	vsnprintf(tmp_buf, sizeof(tmp_buf), message, ap);
	va_end(ap);

	xtrxll_log(log_lvlmap, "USB3380", level, "USB3380: %s\n", tmp_buf);
}

static int xtrxllusb3380v0_open(const char* device, unsigned flags,
								struct xtrxll_base_dev** pdev)
{
	libusb3380_context_t* ctx;
	libusb3380_pcidev_t* pcidev;
	struct xtrxll_usb3380_dev* dev;

	int res;
	const int proto_ver = 0;

	//xtrxll_log_initialize(NULL);
	usb3380_set_logfunc(xtrxllusb3380v0_log, NULL);

	usb3380_set_loglevel(xtrxll_get_loglevel());

	res = usb3380_context_init(&ctx);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "Unable to allocate USB3380 context: error: %d\n", res);
		return res;
	}

	libusb3380_pcie_rc_cfg_t cfg;
	cfg.bar2.addr = DMA_REGION_0_ADDR;
	cfg.bar2.length = BAR_2M;
	for (unsigned i = 0; i < 4; i++) {
		cfg.bar2.qadrants_ep_map[i] = LIBUSB3380_GPEP0;
		cfg.bar2.gpep_in_type[i] = true;
	}
	cfg.bar2.flags = 0;

	cfg.bar3.addr = DMA_REGION_1_ADDR;
	cfg.bar3.length = BAR_2M;
	for (unsigned i = 0; i < 4; i++) {
		cfg.bar3.qadrants_ep_map[i] = LIBUSB3380_GPEP2;
		cfg.bar3.gpep_in_type[i] = false;
	}
	cfg.bar3.flags = 0;
	cfg.gpep_fifo_in_size[0] = 4096;
	cfg.gpep_fifo_in_size[1] = 0;
	cfg.gpep_fifo_in_size[2] = 0;
	cfg.gpep_fifo_in_size[3] = 0;

	cfg.gpep_fifo_out_size[0] = 0;
	cfg.gpep_fifo_out_size[1] = 0;
	cfg.gpep_fifo_out_size[2] = 4096;
	cfg.gpep_fifo_out_size[3] = 0;

	cfg.flags = 0;

	res = usb3380_init_root_complex(ctx, &cfg);
	if (res) {
		if (res == -EAGAIN) {
			usb3380_context_free(ctx);

			res = usb3380_context_init(&ctx);
			if (res) {
				XTRXLL_LOG(XTRXLL_ERROR, "Unable to reinitialize context: error: %d\n", res);
				return res;
			}

			res = usb3380_init_root_complex(ctx, &cfg);
		}
		if (res) {
			XTRXLL_LOG(XTRXLL_ERROR, "Unable to intialize USB3380 Root Complex mode: error: %d\n", res);
			goto usbinit_fail;
		}
	}

	res = usb3380_init_first_dev(ctx, 0, &pcidev);
	if (res) {
		if (res) {
			XTRXLL_LOG(XTRXLL_ERROR, "No devices were found: error: %d\n", res);
			goto usbinit_fail;
		}
	}

	if (usb3380_pci_dev_did(pcidev) != XTRX_DID_V0 ||
			usb3380_pci_dev_vid(pcidev) != XTRX_VID_V0) {
		XTRXLL_LOG(XTRXLL_ERROR, "Enumeared device isn't XTRX [%04x:%04x]\n",
				   usb3380_pci_dev_vid(pcidev),
				   usb3380_pci_dev_did(pcidev));
		goto usbinit_fail;
	}

	dev = (struct xtrxll_usb3380_dev*)malloc(sizeof(struct xtrxll_usb3380_dev));
	if (dev == NULL) {
		res = errno;
		XTRXLL_LOG(XTRXLL_ERROR, "Can't allocate memory for device `%s`: %s\n",
				   device, strerror_safe(res));
		goto failed_malloc;
	}
	dev->base.self = &dev->base;
	dev->base.selfops = xtrxllusb3380v0_init(XTRXLL_ABI_VERSION);
	dev->base.id = dev->pcie_devname;
	dev->ctx = ctx;
	dev->devid = 0x1;
	dev->bar0 = usb3380_pci_dev_bar_addr(pcidev, 0);
	dev->bar1 = usb3380_pci_dev_bar_addr(pcidev, 1);
	dev->rx_dma_flow_ctrl = false;

	res = pthread_mutex_init(&dev->dev_mem_mutex, NULL);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Failed to init mem mutex\n",
				   dev->base.id);
		goto failed_mutex_ctrl;
	}
	dev->rx_queuebuf_ptr = (uint8_t*)((uintptr_t)(dev->queuebuf + EXTRA_DATA_ALIGMENT - 1) & ~((uintptr_t)(EXTRA_DATA_ALIGMENT - 1)));
	dev->tx_queuebuf_ptr = dev->rx_queuebuf_ptr + 2*RXDMA_MMAP_SIZE;

	dev->rx_running = false;
	dev->rx_stop = false;
	res = xtrxllpciebase_init(&dev->pcie);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Failed to init DMA subsystem\n",
				   dev->base.id);
		goto failed_abi_ctrl;
	}
	snprintf(dev->pcie_devname, DEV_NAME_SIZE - 1, "USB3_%d", dev->devid);

	res = xtrxll_base_fill_ctrlops(&dev->base, proto_ver);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Unsupported protocol version: %d",
				   dev->base.id, proto_ver);
		goto failed_abi_ctrl;
	}
	*pdev = &dev->base;

#ifdef ASYNC_MODE
	res = usb3380_async_start(pcidev, &dev->mgr);
	if (res)
		goto failed_async_start;

	res = usb3380_async_set_gpep_timeout(dev->mgr, true, LIBUSB3380_GPEP0, 250);
	if (res)
		goto failed_set_to;

	res = usb3380_async_set_gpep_timeout(dev->mgr, false, LIBUSB3380_GPEP2, 8000);
	if (res)
		goto failed_set_to;

	dev->rx_buf_available = RXDMA_BUFFERS;
	dev->rx_bufno_consumed = 0;
	res = sem_init(&dev->rx_buf_ready, 0, 0);
	if (res)
		goto failed_sem_buf_ready;

	dev->rx_buf_to = false;

	res = sem_init(&dev->rx_gpep_cleared, 0, 0);
	if (res)
		goto failed_sem_gpep_cleared;

	dev->rx_gpep_active = false;

	dev->tx_buf_ready = 0;
	dev->tx_bufno_posted = 0;
	res = sem_init(&dev->tx_buf_available, 0, TXDMA_BUFFERS);
	if (res)
		goto failed_sem_tx_buf_available;

	dev->tx_bufno_consumed = 0;

	dev->tx_gpep_active = false;
	dev->tx_stop = false;
#endif

	// 0 -  128
	// 1 -  256
	// 2 -  512
	// 3 - 1024
	// 4 - 2048
	// 5 - 4096
	/* 256 bytes is supported by usb3380 */
	res = pcieusb3380v0_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_INT_PCIE,
								(1U << INT_PCIE_E_FLAG) |
								(1U << 16) | /* 256 B max_size */
								(4U << 17) | /* 2048 B max_req_size */
								((dev->rx_dma_flow_ctrl ? 0 : 1U) << 24) | /* disable ovf ctrl on RX path */
								(1 << INT_PCIE_I_FLAG) | (1 << 6));
	if (res) {
		goto failed_pcie_cfg;
	}

	uint32_t dummy;
	res = pcieusb3380v0_reg_in(dev, UL_GP_ADDR + GP_PORT_RD_INTERRUPTS, &dummy);
	if (res) {
		goto failed_pcie_cfg;
	}

	res = xtrxllpciebase_dma_start(&dev->pcie, 0, XTRXLL_FE_STOP, XTRXLL_FE_MODE_MIMO,
									0, XTRXLL_FE_STOP, XTRXLL_FE_MODE_MIMO);

	XTRXLL_LOG(XTRXLL_INFO,  "XTRX %s: Device `%s` was opened\n",
			   dev->base.id, device);
	return 0;

failed_pcie_cfg:
#ifdef ASYNC_MODE
	sem_destroy(&dev->tx_buf_available);
failed_sem_tx_buf_available:
	sem_destroy(&dev->rx_gpep_cleared);
failed_sem_gpep_cleared:
	sem_destroy(&dev->rx_buf_ready);
failed_sem_buf_ready:
failed_set_to:
	usb3380_async_stop(dev->mgr);
failed_async_start:
#endif
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

	XTRXLL_LOG(XTRXLL_INFO, "XTRX %d: Device closing\n", dev->devid);

#ifdef ASYNC_MODE
	sem_destroy(&dev->tx_buf_available);
	sem_destroy(&dev->rx_gpep_cleared);
	sem_destroy(&dev->rx_buf_ready);

	usb3380_async_stop(dev->mgr);
#endif
	pthread_mutex_destroy(&dev->dev_mem_mutex);
	usb3380_context_free(dev->ctx);
	free(dev);
}

static int xtrxllusb3380v0_discovery(char* devices, size_t maxbuf)
{
    return -EINVAL;
}

static int xtrxllusb3380v0_dma_rx_init(struct xtrxll_base_dev* bdev, int chan,
									   unsigned buf_szs, unsigned* out_szs)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;

	int res;
	unsigned i;
	if (chan != 0)
		return -EINVAL;

	if (buf_szs % 16 || buf_szs > 2 * RXDMA_MMAP_BUFF) {
		XTRXLL_LOG(XTRXLL_ERROR, "Wire RX pkt size is %d, should be rounded to 128 bit and less %d\n",
				   buf_szs, 2*RXDMA_MMAP_BUFF);
		return -EINVAL;
	} else if (buf_szs == 0) {
		buf_szs = 2 * RXDMA_MMAP_BUFF;
	}

	for (i = 0; i < RXDMA_BUFFERS; i++) {
		uint32_t reg = (((buf_szs / 16) - 1) & 0xFFF) |
				(0xFFFFF000 & (DMA_REGION_1_ADDR + i * 2 * RXDMA_MMAP_BUFF));

		res = pcieusb3380v0_reg_out(dev, UL_RXDMA_ADDR + i, reg);
		if (res)
			return res;
	}

	dev->pcie.cfg_rx_desired_bufsize = buf_szs;
	dev->pcie.cfg_rx_bufsize = buf_szs & ~15U;

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

#ifdef ASYNC_MODE
static int xtrxllusb3380v0_dma_rx_gpep_issue(struct xtrxll_usb3380_dev* dev);

static void xtrxllusb3380v0_dma_rx_gpep_cb(const struct libusb3380_qgpep* gpep)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)gpep->param;
	int res;

	if (gpep->base.status != DQS_SUCCESS) {
		if (dev->rx_stop) {
			dev->rx_gpep_active = false;
			sem_post(&dev->rx_gpep_cleared);
			return;
		}

		XTRXLL_LOG(XTRXLL_ERROR, "GPEP status: %d\n", gpep->base.status);

		res = xtrxllusb3380v0_dma_rx_gpep_issue(dev);
		assert(res == 0);

		if (gpep->base.status == DQS_TIMEOUT) {
			dev->rx_buf_to = false;
		}
		return;
	}

	// issue next URB
	dev->pcie.rd_buf_idx = (dev->pcie.rd_buf_idx + 1) & 0x3f;

	if (!dev->rx_stop) {
		// If there's at least one available buffer, decrement availablity
		// counter and issue next gpep read commad
		unsigned available;
		do {
			available = __atomic_load_n(&dev->rx_buf_available, __ATOMIC_SEQ_CST);
			if (available == 0)
				break;
		} while (!__atomic_compare_exchange_n(&dev->rx_buf_available, &available,
											  available - 1, false,
											  __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST));

		if (available) {
			res = xtrxllusb3380v0_dma_rx_gpep_issue(dev);
			assert(res == 0);

			XTRXLL_LOG(XTRXLL_DEBUG, "ISSUE READ %d buffer (%d avail)\n", dev->pcie.rd_buf_idx, available);
		} else {
			dev->rx_gpep_active = false;
			XTRXLL_LOG(XTRXLL_DEBUG, "NO FREE BUFFER\n");
		}
	} else {
		dev->rx_gpep_active = false;
	}

	// make buffer visible to waiting thread
	res = sem_post(&dev->rx_buf_ready);
	assert(res == 0);
}

int xtrxllusb3380v0_dma_rx_gpep_issue(struct xtrxll_usb3380_dev* dev)
{
	uint8_t* buffer = dev->rx_queuebuf_ptr + (dev->pcie.rd_buf_idx & 0x1f) * 2 * RXDMA_MMAP_BUFF;
	dev->rx_gpep_active = true;
	int res = usb3380_async_gpep_in_post(dev->mgr,
										 LIBUSB3380_GPEP0,
										 buffer,
										 dev->pcie.cfg_rx_bufsize,
										 xtrxllusb3380v0_dma_rx_gpep_cb,
										 dev);
	return res;
}

// Callback from internal IO thread take care to data races

static int xtrxllusb3380v0_dma_rx_getnext(struct xtrxll_base_dev* bdev,
										  int chan, void** addr, wts_long_t *wts,
										  unsigned *sz, unsigned flags)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	int res;

	if (chan != 0)
		return -EINVAL;
	if (dev->rx_stop)
		return -EINTR;

	for (;;) {
		// consume buffer, TODO error checking
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);

		// TODO calculate this size based on pkt arraival time
		ts.tv_nsec += 50 * 1000 * 1000;
		if (ts.tv_nsec > 1000 * 1000 * 1000) {
			ts.tv_nsec -= 1000 * 1000 * 1000;
			ts.tv_sec++;
		}

		res = sem_timedwait(&dev->rx_buf_ready, &ts);
		if (res) {
			if (errno == EINTR)
				continue;

			if (errno != ETIMEDOUT)
				return -EIO;

			if (dev->rx_stop)
				return -EINTR;

			wts_long_t cwts;
			unsigned bn;
			res = xtrxllpciebase_dmarx_get(&dev->pcie, chan, &bn, &cwts, sz,
										   true, 0, true);
			if (res == 0) {
				XTRXLL_LOG(XTRXLL_WARNING, "XTRX RX DATA BUT NOT BUF\n");
				continue;
			} else if (res == -EOVERFLOW) {
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
				XTRXLL_LOG(XTRXLL_WARNING, "XTRX RX AGAIN\n");
				continue;
			} else {
				XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Got %d!\n",
						   dev->base.id, res);
				return res;
			}
		} else {
			break;
		}
	}

	unsigned bufno = (dev->rx_bufno_consumed++) & 0x1f;

	*sz = dev->pcie.cfg_rx_bufsize;
	*addr = dev->rx_queuebuf_ptr + bufno * 2 * RXDMA_MMAP_BUFF;

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

	if (chan != 0)
		return -EINVAL;

	unsigned bufno = ((uint8_t*)addr - dev->rx_queuebuf_ptr) / (2 * RXDMA_MMAP_BUFF);
	assert(bufno < RXDMA_BUFFERS);

	unsigned buffs_available =  __atomic_add_fetch(&dev->rx_buf_available, 1, __ATOMIC_SEQ_CST);
	if (buffs_available == 1 && !dev->rx_stop) {
		// We've got transition from 0 -> 1, so start read process into this buffer
		res = xtrxllusb3380v0_dma_rx_gpep_issue(dev);
	}

	XTRXLL_LOG(XTRXLL_DEBUG,  "XTRX %s: RX DMA RELEASE %d\n",
			   dev->base.id, bufno);
	return res;
}


#else

#define USB3380_READ_TO_MS		1000

static int xtrxllusb3380v0_dma_rx_getnext(struct xtrxll_base_dev* bdev,
										  int chan, void** addr,wts_long_t *wts,
										  unsigned *sz, unsigned flags)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;

	if (chan != 0)
		return -EINVAL;

	//unsigned bufno, bufno_rd;
	int res;
	bool force_log = true;
	for (;;) {
		if (flags & XTRXLL_RX_FORCELOG) {
			force_log = true;
		}
		unsigned off = 0;
		int written = 0;
		for (;;) {
			res = usb3380_gpep_read(dev->ctx, dev->rx_queuebuf_ptr + off,
									dev->pcie.cfg_rx_bufsize,
									&written, USB3380_READ_TO_MS);
			if (res)  {
				XTRXLL_LOG(XTRXLL_ERROR, "Failed: %d (off=%d)\n", res, off);
				break;
			}

			off += written;
			if (off == dev->pcie.cfg_rx_bufsize) {
				XTRXLL_LOG(XTRXLL_DEBUG, "The whole buffer %d\n", off);
				goto done;
			}
		}

		if (res) {
			unsigned bn;
			res = xtrxllpciebase_dmarx_get(&dev->pcie, chan, &bn, wts, sz,
													   force_log, 0);
			if (res == 0) {
				//Should not happen!
				XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Data ready but not EP! bn=%d\n",
						   dev->base.id, bn);
				return -EPIPE;
			} else if (res == -EOVERFLOW) {
				if (!(flags & XTRXLL_RX_NOSTALL)) {
					return -EOVERFLOW;
				}
				if (dev->rx_stop)
					return -EOVERFLOW;

				xtrxllpciebase_dmarx_resume(&dev->pcie, chan, *wts);
			} else if (res == -EAGAIN) {
				continue;
			} else {
				XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Got %d!\n",
						   dev->base.id, res);
				goto transfer_err;
			}
		}
	}

done:
	/* we've already cleared out EP, so we can release the buffer on PCIe side */
	pcieusb3380v0_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_RXDMA_CNFRM, 0);

	/* we've got a valid RX buffer */
	if (wts) {
		*wts = dev->pcie.rd_cur_sample;
		dev->pcie.rd_cur_sample += dev->pcie.rd_block_samples;
	}

	dev->pcie.rd_buf_idx = (dev->pcie.rd_buf_idx + 1) & 0x3f;

	*sz = dev->pcie.cfg_rx_bufsize;
	*addr = dev->rx_queuebuf_ptr;
	return 0;

transfer_err:
	return res;
}

static int xtrxllusb3380v0_dma_rx_release(struct xtrxll_base_dev* bdev,
										  int chan, void* addr)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;

	XTRXLL_LOG(XTRXLL_DEBUG,  "XTRX %s: RX DMA RELEASE\n",
			   dev->base.id);
	return 0;
}
#endif


// TX DMA

static int xtrxllusb3380v0_dma_tx_init(struct xtrxll_base_dev* bdev, int chan,
									   unsigned buf_szs)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;

	if (chan != 0)
		return -EINVAL;

	int res;
	unsigned i;
	for (i = 0; i < TXDMA_BUFFERS; i++) {
		uint32_t reg = (((buf_szs / 16) - 1) & 0xFFF) |
				(0xFFFFF000 & (DMA_REGION_0_ADDR + i * TXDMA_MMAP_BUFF));

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

static int xtrxllusb3380v0_dma_tx_gpep_issue(struct xtrxll_usb3380_dev* dev, unsigned available);

static void xtrxllusb3380v0_dma_tx_gpep_cb(const struct libusb3380_qgpep* gpep)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)gpep->param;
	int res;

	assert(dev->tx_gpep_active);
	XTRXLL_LOG(XTRXLL_DEBUG, "WRITE DONE ( %d pkts ) %d RDY: %d\n",
			   dev->tx_bufs_in_transfer, dev->tx_bufno_consumed,
			   dev->tx_buf_ready);

	// make buffer available for filling again thread
	unsigned pkts = dev->tx_bufs_in_transfer;
	while (pkts-- > 0) {
		res = sem_post(&dev->tx_buf_available);
		assert(res == 0);
	}

	unsigned available = __atomic_sub_fetch(&dev->tx_buf_ready, dev->tx_bufs_in_transfer, __ATOMIC_SEQ_CST);
	if (gpep->base.status != DQS_SUCCESS) {
		dev->tx_gpep_active = false;

		if (dev->tx_stop) {
			return;
		}

		XTRXLL_LOG(XTRXLL_ERROR, "GPEP_OUT status: %d\n", gpep->base.status);

		// TODO handle TX timeout properly
		abort();
	}

	if (!dev->tx_stop) {
		if (available) {
			res = xtrxllusb3380v0_dma_tx_gpep_issue(dev, available);
			assert(res == 0);

			XTRXLL_LOG(XTRXLL_DEBUG, "ISSUE WRITE %d buffer (%d avail)\n",
					   dev->tx_bufno_consumed - 1, available);
		} else {
			dev->tx_gpep_active = false;
			XTRXLL_LOG(XTRXLL_DEBUG, "TX NO FREE BUFFER\n");
		}
	} else {
		dev->tx_gpep_active = false;
	}

}


int xtrxllusb3380v0_dma_tx_gpep_issue(struct xtrxll_usb3380_dev* dev, unsigned available)
{
	uint8_t* buffer = dev->tx_queuebuf_ptr + (dev->tx_bufno_consumed & 0x1f) * TXDMA_MMAP_BUFF;
	unsigned length = dev->tx_post_size[dev->tx_bufno_consumed & 0x1f];

	dev->tx_bufs_in_transfer = 1;
	dev->tx_gpep_active = true;

	unsigned i;
	if (length == TXDMA_MMAP_BUFF && ((dev->tx_bufno_consumed & 0x1f) != 0x1f)) {
		for (i = 1; (i < available) && (i < 1); i++) {
			unsigned nxt_len = dev->tx_post_size[(dev->tx_bufno_consumed + i) & 0x1f];
			length += nxt_len;
			dev->tx_bufs_in_transfer++;

			if (nxt_len != TXDMA_MMAP_BUFF)
				break;
			if (((dev->tx_bufno_consumed + i) & 0x1f) == 0x1f)
				break;
		}
	}

	dev->tx_bufno_consumed += dev->tx_bufs_in_transfer ;

	assert(dev->tx_bufs_in_transfer > 0 && dev->tx_bufs_in_transfer <= TXDMA_BUFFERS);
	int res = usb3380_async_gpep_out_post(dev->mgr,
										  LIBUSB3380_GPEP2,
										  buffer,
										  length,
										  xtrxllusb3380v0_dma_tx_gpep_cb,
										  dev);
	return res;
}

static int xtrxllusb3380v0_dma_tx_getfree_ex(struct xtrxll_base_dev* bdev,
											 int chan, void** addr,
											 uint16_t* late)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	int res;
	int ilate = 0;
	unsigned bufno = TXDMA_MMAP_BUFF; // should not happed

	if (chan != 0)
		return -EINVAL;

	if (addr == NULL) {
		res = xtrxllpciebase_dmatx_get(&dev->pcie, chan, NULL, &ilate);
		if (late) {
			*late = ilate;
		}
		return res;
	}

	bool bufno_obtained = false;
	bool got_buf = false;

	for (;;) {
		// consume buffer, TODO error checking
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);

		ts.tv_nsec += 100 * 1000 * 1000;
		if (ts.tv_nsec > 1000 * 1000 * 1000) {
			ts.tv_nsec -= 1000 * 1000 * 1000;
			ts.tv_sec++;
		}

		res = sem_timedwait(&dev->tx_buf_available, &ts);
		if (res) {
			if (errno == EINTR)
				continue;

			if (errno != ETIMEDOUT)
				return -EIO;
		} else {
			got_buf = true;
		}

		if (dev->tx_stop) {
			return -EIO;
		}

		if (!bufno_obtained) {
			res = xtrxllpciebase_dmatx_get(&dev->pcie, chan, &bufno, &ilate);
			if (res) {
				if (res == -EBUSY) {
					XTRXLL_LOG(XTRXLL_ERROR,  "XTRX %s: TX BUSY\n",
							   dev->base.id);

					xtrxllpciebase_dmatx_get(&dev->pcie, chan, NULL, &ilate);
				} else if (res == 0) {
					goto sec_ok;
				} else {
					return res;
				}
				continue;
			}
sec_ok:
			assert(bufno == (dev->tx_bufno_posted & 0x1f));
			dev->tx_bufno_posted++;

			bufno_obtained = true;
		}

		if (got_buf)
			break;
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

	res = xtrxllpciebase_dmatx_post(&dev->pcie, chan, bufno,
									wts, samples);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR,  "XTRX %s: TX POST failed buf %d: error %d (written %d)\n",
				   dev->base.id, bufno, res, written);
		return res;
	}


	dev->tx_post_size[bufno] = samples * 8;

	unsigned buffs_available =  __atomic_add_fetch(&dev->tx_buf_ready, 1, __ATOMIC_SEQ_CST);
	if (buffs_available == 1 && !dev->tx_stop) {
		// We've got transition from 0 -> 1, so start out process from this buffer
		res = xtrxllusb3380v0_dma_tx_gpep_issue(dev, 1);

		XTRXLL_LOG(XTRXLL_ERROR, "ISSUE FROM CTRL\n");
	} else {
		res = 0;
	}
	return res;
}

#if 0
static int xtrxllusb3380v0_dma_tx_getfree_ex(struct xtrxll_base_dev* bdev,
											 int chan, void** addr,
											 uint16_t* late)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	unsigned bufno = TXDMA_MMAP_BUFF; // should not happed
	int ilate;
	unsigned *pbufno = (addr) ? &bufno : NULL;
	int res = xtrxllpciebase_dmatx_get(&dev->pcie, chan, pbufno, &ilate);
	if (res) {
		if (res == -EBUSY) {
			XTRXLL_LOG(XTRXLL_ERROR,  "XTRX %s: TX BUSY\n",
					   dev->base.id);

			xtrxllpciebase_dmatx_get(&dev->pcie, chan, NULL, &ilate);
		}
		return res;
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

	res = xtrxllpciebase_dmatx_post(&dev->pcie, chan, bufno,
									wts, samples);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR,  "XTRX %s: TX POST failed buf %d: error %d (written %d)\n",
				   dev->base.id, bufno, res, written);
		return res;
	}

#ifndef ASYNC_MODE
	res = usb3380_gpep_write(dev->ctx, (const uint8_t*)addr,
							 samples * 8, &written, 8050);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR,  "XTRX %s: TX GPEP failed buf %d: error %d (written %d)\n",
				   dev->base.id, bufno, res, written);
		return -EIO;
	}
#endif
#ifdef ASYNC_MODE
	res = usb3380_async_gpep_out_post(dev->mgr, (const uint8_t*)addr,
								 samples * 8);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR,  "XTRX %s: TX GPEP failed buf %d: error %d (written %d)\n",
				   dev->base.id, bufno, res, written);
		return -EIO;
	}
#endif

	return 0;
}
#endif

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

static int xtrxllusb3380v0_dma_start(struct xtrxll_base_dev* bdev, int chan,
									 xtrxll_fe_t rxfe, xtrxll_mode_t rxmode,
									 wts_long_t rx_start_sample,
									 xtrxll_fe_t txfe, xtrxll_mode_t txmode)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;
	dev->tx_chans = (txmode == XTRXLL_FE_MODE_MIMO) ? 2 : 1;

	int res;

	if (rxfe == XTRXLL_FE_STOP) {
		dev->rx_stop = true;
		res = pcieusb3380v0_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_RXTXDMA,
									(1UL << GP_PORT_WR_RXTXDMA_RXV) | (rxfe << GP_PORT_WR_RXTXDMA_RXOFF));

#ifndef ASYNC_MODE
		uint8_t tmpbuf[32768];
		int written;
		res = usb3380_gpep_read(dev->ctx, tmpbuf,
								sizeof(tmpbuf),
								&written, 1);
#endif
	} else if (rxfe != XTRXLL_FE_DONTTOUCH) {
#ifdef ASYNC_MODE
		dev->rx_stop = false;
		dev->rx_buf_to = false;
		dev->rx_bufno_consumed = 0;

		dev->rx_buf_available = RXDMA_BUFFERS;
		sem_destroy(&dev->rx_buf_ready);
		sem_init(&dev->rx_buf_ready, 0, 0);
#endif
	}

#ifdef ASYNC_MODE
	if (txfe == XTRXLL_FE_STOP) {
		dev->rx_stop = true;
		while (dev->tx_gpep_active) {
			usleep(1000);
		}
	} else if (txfe != XTRXLL_FE_DONTTOUCH) {
		dev->rx_stop = false;
		//dev->tx_buf_available = TXDMA_BUFFERS;
		//if (sem_getvalue(&dev->tx_buf_available) != TXDMA_BUFFERS)
		sem_destroy(&dev->tx_buf_available);
		sem_init(&dev->tx_buf_available, 0, TXDMA_BUFFERS);

		dev->tx_bufno_consumed = 0;
		dev->tx_bufno_posted = 0;
		dev->tx_buf_ready = 0;
	}
#endif

	res = xtrxllpciebase_dma_start(&dev->pcie, chan, rxfe, rxmode,
									rx_start_sample, txfe, txmode);

#ifdef ASYNC_MODE
	if (rxfe == XTRXLL_FE_STOP) {
		if (dev->rx_gpep_active) {
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
		xtrxllusb3380v0_dma_rx_gpep_issue(dev);
	}
#endif

	return res;
}

static int xtrxllusb3380v0_get_sensor(struct xtrxll_base_dev* bdev,
									  unsigned sensorno, int* outval)
{
	struct xtrxll_usb3380_dev* dev = (struct xtrxll_usb3380_dev*)bdev;

	switch (sensorno) {
	case XTRXLL_DMABUF_RXST64K:
		*outval = (RXDMA_BUFFERS - dev->rx_buf_available) * 65536 / RXDMA_BUFFERS;
		return 0;
	case XTRXLL_DMABUF_TXST64K:
		sem_getvalue(&dev->tx_buf_available, outval);
		*outval = (TXDMA_BUFFERS - *outval) * 65536 / TXDMA_BUFFERS;
		return 0;
	default:
		return bdev->ctrlops->get_sensor(bdev->self, sensorno, outval);
	}
}

static int xtrxllusb3380v0_set_param(struct xtrxll_base_dev* dev,
									  unsigned paramno, unsigned val)
{
	return dev->ctrlops->set_param(dev->self, paramno, val);
}

const static struct xtrxll_ops mod_ops = {
	.open = xtrxllusb3380v0_open,
	.close = xtrxllusb3380v0_close,
	.discovery = xtrxllusb3380v0_discovery,

	.reg_out = xtrxllusb3380v0_reg_out,
	.reg_in = xtrxllusb3380v0_reg_in,

	.reg_out_n = xtrxllusb3380v0_reg_out_n,
	.reg_in_n = xtrxllusb3380v0_reg_in_n,

	.spi_bulk = xtrxllusb3380v0_lms7_spi_bulk,

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

const struct xtrxll_ops* xtrxll_init(unsigned abi_version)
{
	return xtrxllusb3380v0_init(abi_version);
}
