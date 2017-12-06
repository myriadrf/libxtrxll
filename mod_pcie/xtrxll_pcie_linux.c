/*
 * xtrx low level PCIe module source file
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
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <inttypes.h>

#include "xtrxll_log.h"
#include "xtrxll_api.h"
#include "xtrxll_base.h"
#include "xtrxll_base_pcie.h"
#include "xtrxll_pcie_linux.h"

/* Include constants used in kernel mode driver */
#include "xtrx_defs.h"

/* Include verilog file used for FPGA build */
#define localparam static const unsigned
#include "xtrxll_regs.vh"

/* move into xtrx_defs.h */
enum xtrxll_mmaps {
	XTRXLL_MMAP_CONFREGS_OFF = 0,
	XTRXLL_MMAP_CONFREGS_LEN = 4096,
};

#define DEV_NAME_SIZE      32
struct xtrxll_pcie_dev
{
	union {
		struct xtrxll_base_dev base;
		struct xtrxll_base_pcie_dma pcie;
	};

	char pcie_devname[DEV_NAME_SIZE];

	int fd;

	volatile uint32_t* mmap_xtrxll_regs;

	void* mmap_tx_kernel_buf;
	void* mmap_rx_kernel_buf;

	void* mmap_tx_device_buf;
	void* mmap_stat_buf;
};

const struct xtrxll_ops* xtrxll_init(unsigned abi_version);

///////////////////////////////////////////////
// Library-wide variables

static char* strerror_safe(int err)
{
	static __thread char tmp_buffer[512];
	tmp_buffer[0] = 0;

	strerror_r(err, tmp_buffer, sizeof(tmp_buffer));
	return tmp_buffer;
}

static void internal_xtrxll_reg_out_n(struct xtrxll_pcie_dev* dev,
									  unsigned streg, const uint32_t* outval,
									  unsigned count)
{
	unsigned i;
	uint32_t to_write[count];
	for (i = 0; i < count; i++) {
		to_write[i] = htobe32(outval[i]);
	}

	memcpy((void*)&dev->mmap_xtrxll_regs[streg], to_write, count * 4);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);

	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Write [%04x+%d] = %08x\n",
			   dev->base.id, streg, count, outval[0]);
}

static void internal_xtrxll_reg_in_n(struct xtrxll_pcie_dev* dev,
									 unsigned streg, uint32_t* inval,
									 unsigned count)
{
	unsigned i;
	uint32_t to_read[count];

	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	memcpy(to_read, (void*)&dev->mmap_xtrxll_regs[streg], count * 4);

	for (i = 0; i < count; i++) {
		inval[i] = be32toh(to_read[i]);
	}

	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Read [%04x+%d] = %08x\n",
			   dev->base.id, streg, count, inval[0]);
}

static void internal_xtrxll_reg_out(struct xtrxll_pcie_dev* dev, unsigned reg,
									uint32_t outval)
{
	__atomic_store_n(&dev->mmap_xtrxll_regs[reg], htobe32(outval),
					 /*__ATOMIC_RELEASE*/ __ATOMIC_SEQ_CST);

	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Write [%04x] = %08x\n",
			   dev->base.id, reg, outval);
}

static uint32_t internal_xtrxll_reg_in(struct xtrxll_pcie_dev* dev, unsigned reg)
{
	uint32_t val = be32toh(__atomic_load_n(&dev->mmap_xtrxll_regs[reg],
										   /*__ATOMIC_ACQUIRE*/ __ATOMIC_SEQ_CST));

	XTRXLL_LOG(XTRXLL_DEBUG_REGS, "XTRX %s: Read  [%04x] = %08x\n",
			   dev->base.id, reg, val);
	return val;
}

static void internal_xtrxll_transact_spi_wr(struct xtrxll_pcie_dev* dev, uint32_t value)
{
	internal_xtrxll_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_SPI_LMS7_0, value);
}

static uint32_t internal_xtrxll_transact_spi_rb(struct xtrxll_pcie_dev* dev)
{
	return internal_xtrxll_reg_in(dev, UL_GP_ADDR + GP_PORT_RD_SPI_LMS7_0);
}

static int xtrxllpciev0_lms7_spi_bulk(struct xtrxll_base_dev* bdev,
									  uint32_t lmsno, const uint32_t* out,
									  uint32_t* in, size_t count)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;

	if ((lmsno & XTRXLL_LMS7_0) == 0) {
		return -EINVAL;
	}

	unsigned i;
	for (i = 0; i < count; ++i) {
		internal_xtrxll_transact_spi_wr(dev, out[i]);

		ssize_t icnt;
		do {
			icnt = pread(dev->fd, NULL, 0, XTRX_KERN_MMAP_CTRL_IRQS);
			if (icnt < 0) {
				int err = errno;
				if (err != -EAGAIN) {
					XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: SPI IRQ error %d\n",
							   dev->base.id, err);
					return -err;
				}
			}

			XTRXLL_LOG(XTRXLL_DEBUG, "XTRX %s: SPI[%d/%d] I:%d\n",
					   dev->base.id, i, (unsigned)count, (int)icnt);
		} while (icnt != 1);

		in[i] = internal_xtrxll_transact_spi_rb(dev);

		XTRXLL_LOG(XTRXLL_DEBUG, "XTRX %s: SPI[%d/%d] %08x => %08x\n",
				   dev->base.id, i, (unsigned)count, out[i], in[i]);
	}
	return 0;
}

static int xtrxllpciev0_open(const char* device, unsigned flags,
							 struct xtrxll_base_dev** pdev)
{
	xtrxll_log_initialize(NULL);

	int err;
	int fd = open(device, O_RDWR);
	if (fd < 0) {
		err = errno;
		XTRXLL_LOG(XTRXLL_ERROR, "Can't open device `%s`: %s\n",
				   device, strerror_safe(err));
		goto failed_open;
	}

	void* mem = mmap(0, XTRXLL_MMAP_CONFREGS_LEN, PROT_READ | PROT_WRITE,
					 MAP_SHARED, fd, XTRXLL_MMAP_CONFREGS_OFF);
	if (mem == MAP_FAILED) {
		err = errno;
		XTRXLL_LOG(XTRXLL_ERROR, "Can't mmap config area for device `%s`: %s\n",
				   device, strerror_safe(err));
		goto failed_mmap;
	}

	void* mem_stat = mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 4096*1024);
	if (mem_stat == MAP_FAILED) {
		err = errno;
		XTRXLL_LOG(XTRXLL_ERROR, "Can't mmap stat area for device `%s`: %s\n",
				   device, strerror_safe(err));
		goto failed_mmap2;
	}

	struct xtrxll_pcie_dev* dev;
	dev = (struct xtrxll_pcie_dev*)malloc(sizeof(struct xtrxll_pcie_dev));
	if (dev == NULL) {
		err = errno;
		XTRXLL_LOG(XTRXLL_ERROR, "Can't allocate memory for device `%s`: %s\n",
				   device, strerror_safe(err));
		goto failed_malloc;
	}

	dev->base.self = &dev->base;
	dev->base.selfops = xtrxllpciev0_init(XTRXLL_ABI_VERSION);
	dev->base.id = dev->pcie_devname;
	dev->fd = fd;
	dev->mmap_xtrxll_regs = (volatile uint32_t* )mem;
	dev->mmap_rx_kernel_buf = NULL;
	dev->mmap_tx_kernel_buf = NULL;
	dev->mmap_tx_device_buf = NULL;
	dev->mmap_stat_buf = mem_stat;
	err = xtrxllpciebase_init(&dev->pcie);
	if (err) {
		XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Failed to init DMA subsystem\n",
				   dev->base.id);
		goto failed_abi_ctrl;
	}
	snprintf(dev->pcie_devname, DEV_NAME_SIZE - 1, "PCI_%d", fd);

	const int proto_ver = 0;
	err = xtrxll_base_fill_ctrlops(&dev->base, proto_ver);
	if (err) {
		XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Unsupported protocol version: %d",
				   dev->base.id, proto_ver);
		goto failed_abi_ctrl;
	}
	*pdev = &dev->base;

	internal_xtrxll_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_INT_PCIE,
							(1U << INT_PCIE_E_FLAG) |
							(0U << 16) | /* 128 B max_size */
							(4U << 17) | /* 2048 B max_req_size */
							/*0x80F80000 |*/ (1 << INT_PCIE_I_FLAG) | ( 0xfff /* & (~(1 << INT_L_GPS_UART_RX)) & (~(1 << INT_L_GPS_UART_TX)) */ ) /*(0x7 << 6) | 0x3f*/ );

	// TODO make ioctl for clearing interrupt states to be coherent with the
	// driver state
	// internal_xtrxll_reg_in(dev, UL_GP_ADDR + GP_PORT_RD_INTERRUPTS);

	XTRXLL_LOG(XTRXLL_INFO,  "XTRX %s: Device `%s` was opened\n",
			   dev->base.id, device);
	return 0;

failed_abi_ctrl:
failed_malloc:
	munmap(mem_stat, XTRXLL_MMAP_CONFREGS_LEN);
failed_mmap2:
	munmap(mem, XTRXLL_MMAP_CONFREGS_LEN);
failed_mmap:
	close(fd);
failed_open:
	return -err;
}

static void xtrxllpciev0_close(struct xtrxll_base_dev* bdev)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;

	XTRXLL_LOG(XTRXLL_INFO,  "XTRX %s: Device closing\n", dev->base.id);

	munmap((void*)dev->mmap_xtrxll_regs, XTRXLL_MMAP_CONFREGS_LEN);
	munmap((void*)dev->mmap_stat_buf, XTRXLL_MMAP_CONFREGS_LEN);
	close(dev->fd);
	free(dev);
}

static int xtrxllpciev0_discovery(char* devices, size_t maxbuf)
{
	return -EINVAL;
}

static int xtrxllpciev0_dma_rx_init(struct xtrxll_base_dev* bdev, int chan,
									unsigned buf_szs, unsigned* out_szs)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;

	int err;
	if (chan !=0)
		return -EINVAL;
	if (buf_szs % 16 || buf_szs > RXDMA_MMAP_BUFF) {
		XTRXLL_LOG(XTRXLL_ERROR, "Wire RX pkt size is %d, should be rounded to 128 bit and less %d\n",
				   buf_szs, RXDMA_MMAP_BUFF);
		return -EINVAL;
	} else if (buf_szs == 0) {
		buf_szs = RXDMA_MMAP_BUFF;
	}

	if (dev->mmap_rx_kernel_buf == 0) {
		void* m = mmap(0, RXDMA_MMAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
					   dev->fd, XTRX_MMAP_RX_OFF);
		if (m == MAP_FAILED) {
			err = errno;
			XTRXLL_LOG(XTRXLL_ERROR,  "XTRX %s: DMA RX mmap*() failed: %s\n",
					   dev->base.id, strerror_safe(err));
			goto failed;
		}
		XTRXLL_LOG(XTRXLL_DEBUG,  "XTRX %s: DMA RX mmaped to %p\n", dev->base.id, m);
		dev->mmap_rx_kernel_buf = m;
	}
	err = ioctl(dev->fd, 0x123459, buf_szs / 16);
	if (err) {
		XTRXLL_LOG(XTRXLL_ERROR,  "XTRX %s: Unable to set desired buffer size %d\n",
				   dev->base.id, buf_szs);
		return -EFAULT;
	}
	dev->pcie.cfg_rx_desired_bufsize = buf_szs;
	dev->pcie.cfg_rx_bufsize = buf_szs & ~15U;

	*out_szs = dev->pcie.cfg_rx_bufsize;
	return 0;
failed:
	return -err;
}

static int xtrxllpciev0_dma_rx_deinit(struct xtrxll_base_dev* bdev, int chan)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;

	int err;
	if (chan !=0)
		return -EINVAL;
	err = munmap(dev->mmap_rx_kernel_buf, RXDMA_MMAP_SIZE);
	if (err) {
		err = errno;
		XTRXLL_LOG(XTRXLL_DEBUG,  "XTRX %s: DMA RX unmmap error: %s\n",
				   dev->base.id, strerror_safe(err));
		return -err;
	} else {
		XTRXLL_LOG(XTRXLL_DEBUG,  "XTRX %s: DMA RX unmmaped\n",
				   dev->base.id);
	}
	return 0;
}

static int xtrxllpciev0_dma_rx_resume_at(struct xtrxll_base_dev *bdev, int chan, wts_long_t nxt)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	return xtrxllpciebase_dmarx_resume(&dev->pcie, chan, nxt);
}

//                                   1                      1             12              1           1        2         2           6               6
// assign axis_stat_data = { ring_overflow, ring_enough_for_pcie, dma_block_rem, dma_blk_ready, fe_reset, fe_mode, fe_fmt, dma_bufno_read, dma_bufno_reg };

//localparam RXDMA_STAT_BUFNO_FILL = 0;
//localparam RXDMA_STAT_BUFNO_READ = 6;

static int xtrxllpciev0_dma_rx_getnext(struct xtrxll_base_dev* bdev, int chan,
									   void** addr, wts_long_t *wts, unsigned *sz,
									   unsigned flags)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;

	unsigned bn;
	int res;
	bool force_log = false;

	uint32_t icnt = __atomic_exchange_n((uint32_t*)dev->mmap_stat_buf + XTRX_KERN_MMAP_RX_IRQS,
										0, __ATOMIC_SEQ_CST);

	for (;;) {
		//		uint32_t icnt = __atomic_exchange_n((int32_t*)dev->mmap_stat_buf + XTRX_KERN_MMAP_RX_IRQS,
		//											0, __ATOMIC_SEQ_CST);

		if ((flags & XTRXLL_RX_FORCELOG) || ((flags & XTRXLL_RX_SPURSINTLOG) && (icnt > 1))) {
			force_log = true;
		}

		res = xtrxllpciebase_dmarx_get(&dev->pcie, chan, &bn, wts, sz,
									   force_log, icnt, false);
		if (res == 0) {
			break;
		} else if (res == -EOVERFLOW) {
			if (!(flags & XTRXLL_RX_NOSTALL)) {
				return -EOVERFLOW;
			}

			xtrxllpciebase_dmarx_resume(&dev->pcie, chan, *wts);
		} else if (res == -EAGAIN) {
			if (flags & XTRXLL_RX_DONTWAIT) {
				return -EAGAIN;
			}
#if 0
			// Wait for event
			struct pollfd pfd;
			int err;

			pfd.fd = dev->fd;
			pfd.events = POLLIN | POLLRDBAND;
			pfd.revents = 0;

			err = poll(&pfd, 1, 10000);
			if (err < 0) {
				XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: RX DMA error %d\n",
						   dev->base.id, errno);
				return -EFAULT;
			} else if (err == 0) {
				XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: RX DMA timed out\n",
						   dev->base.id);
				force_log = true;
			}
#endif
			ssize_t err = pread(dev->fd, NULL, 0, XTRX_KERN_MMAP_RX_IRQS);
			if (err < 0) {
				if (errno != -EAGAIN) {
					XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: RX DMA error %d\n",
							   dev->base.id, errno);
					return -EFAULT;
				}
				icnt = 0;
			} else {
				icnt = err;
			}
		} else {
			XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Got %d!\n",
					   dev->base.id, res);
			return res;
		}
	}

	*addr = (void*)((char*)dev->mmap_rx_kernel_buf + RXDMA_MMAP_BUFF * bn);
	return 0;
}

static int xtrxllpciev0_dma_rx_release(struct xtrxll_base_dev* bdev, int chan,
									   void* addr)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	if (chan != 0)
		return -EINVAL;

	unsigned bufno = ((char*)addr - (char*)dev->mmap_rx_kernel_buf) / RXDMA_MMAP_BUFF;
	XTRXLL_LOG(XTRXLL_DEBUG,  "XTRX %s: RX DMA RELEASE %d\n", dev->base.id, bufno);

	if (bufno > 0x1f)
		return -EINVAL;

	internal_xtrxll_reg_out(dev, UL_GP_ADDR + GP_PORT_WR_RXDMA_CNFRM, bufno);
	return 0;
}

// TX DMA

static int xtrxllpciev0_dma_tx_init(struct xtrxll_base_dev* bdev, int chan,
									unsigned buf_szs)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	int err;
	if (chan !=0)
		return -EINVAL;
	if (dev->mmap_tx_kernel_buf != 0)
		return -EBUSY;

	void* m = mmap(0, TXDMA_MMAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
				   dev->fd, XTRX_MMAP_TX_OFF);
	if (m == MAP_FAILED) {
		err = -errno;
		XTRXLL_LOG(XTRXLL_ERROR,  "XTRX %s: DMA TX mmap*() failed: %s\n",
				   dev->base.id, strerror_safe(err));
		goto failed;
	}

	XTRXLL_LOG(XTRXLL_DEBUG,  "XTRX %s: DMA TX mmaped to %p\n",
			   dev->base.id, m);
	dev->mmap_tx_kernel_buf = m;
	return 0;
failed:
	return -err;
}

static int xtrxllpciev0_dma_tx_deinit(struct xtrxll_base_dev* bdev, int chan)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	int err;
	if (chan !=0)
		return -EINVAL;

	err = munmap(dev->mmap_tx_kernel_buf, TXDMA_MMAP_SIZE);
	if (err) {
		err = errno;
		XTRXLL_LOG(XTRXLL_DEBUG,  "XTRX %s: DMA TX unmmap error: %s\n",
				   dev->base.id ,strerror_safe(err));
		return -err;
	} else {
		XTRXLL_LOG(XTRXLL_DEBUG,  "XTRX %s: DMA TX unmmaped\n",
				   dev->base.id);
	}
	return 0;
}

static int xtrxllpciev0_dma_tx_getfree_ex(struct xtrxll_base_dev* bdev,
										  int chan, void** addr, uint16_t* late)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	unsigned bufno;
	int ilate;
	unsigned *pbufno = (addr) ? &bufno : NULL;
	int res = xtrxllpciebase_dmatx_get(&dev->pcie, chan, pbufno, &ilate);
	if (res)
		return res;

	if (late) {
		*late = ilate;
	}
	if (addr) {
		*addr = (void*)((char*)dev->mmap_tx_kernel_buf + TXDMA_MMAP_BUFF * bufno);
	}
	return 0;
}

static int xtrxllpciev0_dma_tx_post(struct xtrxll_base_dev* bdev, int chan,
									void* addr, wts_long_t wts,
									uint32_t samples)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	unsigned bufno = ((char*)addr - (char*)dev->mmap_tx_kernel_buf) / TXDMA_MMAP_BUFF;
	return xtrxllpciebase_dmatx_post(&dev->pcie, chan, bufno, wts, samples);
}


static int xtrxllpciev0_repeat_tx_buf(struct xtrxll_base_dev* bdev, int chan,
									  xtrxll_fe_t fmt, const void* buff,
									  unsigned buf_szs, xtrxll_mode_t mode)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	int err;

	if (dev->mmap_tx_device_buf == 0) {
		void* m = mmap(0, TXDMA_MMAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED,
					   dev->fd, XTRX_MMAP_TX_BUF_OFF);
		if (m == MAP_FAILED) {
			err = errno;
			XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: DMA RX mmap*() failed: %s\n",
					   dev->base.id, strerror_safe(err));
			goto failed;
		}

		XTRXLL_LOG(XTRXLL_DEBUG, "XTRX %s: DMA RX mmaped to %p\n",
				   dev->base.id, m);
		dev->mmap_tx_device_buf = m;
	}

	err = xtrxllpciebase_repeat_tx(&dev->pcie, chan, fmt, buf_szs, mode);
	if (err)
		return err;

	memcpy(dev->mmap_tx_device_buf, buff, buf_szs);
	return 0;
failed:
	return -err;
}

static int xtrxllpciev0_repeat_tx_start(struct xtrxll_base_dev* bdev,
										int chan, int start)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	return xtrxllpciebase_repeat_tx_start(&dev->pcie, chan, start);
}

static int xtrxllpciev0_dma_start(struct xtrxll_base_dev* bdev, int chan,
								  xtrxll_fe_t rxfe, xtrxll_mode_t rxmode,
								  wts_long_t rx_start_sample,
								  xtrxll_fe_t txfe, xtrxll_mode_t txmode)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	return xtrxllpciebase_dma_start(&dev->pcie, chan, rxfe, rxmode,
									rx_start_sample, txfe, txmode);
}

static int xtrxllpciev0_reg_out(struct xtrxll_base_dev* bdev, unsigned reg,
								uint32_t outval)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	internal_xtrxll_reg_out(dev, reg, outval);
	return 0;
}

static int xtrxllpciev0_reg_in(struct xtrxll_base_dev* bdev, unsigned reg,
							   uint32_t* inval)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	*inval = internal_xtrxll_reg_in(dev, reg);
	return 0;
}

static int xtrxllpciev0_reg_out_n(struct xtrxll_base_dev* bdev, unsigned streg,
								  const uint32_t* outval, unsigned count)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	internal_xtrxll_reg_out_n(dev, streg, outval, count);
	return 0;
}

static int xtrxllpciev0_reg_in_n(struct xtrxll_base_dev* bdev, unsigned streg,
								 uint32_t* inval, unsigned count)
{
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	internal_xtrxll_reg_in_n(dev, streg, inval, count);
	return 0;
}

static int xtrxllpciev0_get_sensor(struct xtrxll_base_dev* bdev,
								   unsigned sensorno, int* outval)
{
	uint32_t reg;
	struct xtrxll_pcie_dev* dev = (struct xtrxll_pcie_dev*)bdev;
	switch (sensorno) {
	case XTRXLL_ONEPPS_CAPTURED:
		//FIXME!!!!
		*outval = pread(dev->fd, &reg, sizeof(reg), XTRX_KERN_MMAP_1PPS_IRQS);
		return ( *outval < 0 ) ? *outval : 0;
	default:
		break;
	}

	return bdev->ctrlops->get_sensor(bdev->self, sensorno, outval);
}

const static struct xtrxll_ops mod_ops = {
	.open = xtrxllpciev0_open,
	.close = xtrxllpciev0_close,
	.discovery = xtrxllpciev0_discovery,

	.reg_out = xtrxllpciev0_reg_out,
	.reg_in = xtrxllpciev0_reg_in,

	.reg_out_n = xtrxllpciev0_reg_out_n,
	.reg_in_n = xtrxllpciev0_reg_in_n,

	.spi_bulk = xtrxllpciev0_lms7_spi_bulk,

	// RX DMA
	.dma_rx_init = xtrxllpciev0_dma_rx_init,
	.dma_rx_deinit = xtrxllpciev0_dma_rx_deinit,

	.dma_rx_getnext = xtrxllpciev0_dma_rx_getnext,
	.dma_rx_release = xtrxllpciev0_dma_rx_release,

	.dma_rx_resume_at = xtrxllpciev0_dma_rx_resume_at,

	// TX DMA
	.dma_tx_init = xtrxllpciev0_dma_tx_init,
	.dma_tx_deinit = xtrxllpciev0_dma_tx_deinit,
	.dma_tx_getfree_ex = xtrxllpciev0_dma_tx_getfree_ex,
	.dma_tx_post = xtrxllpciev0_dma_tx_post,
	.dma_start = xtrxllpciev0_dma_start,

	.repeat_tx_buf = xtrxllpciev0_repeat_tx_buf,
	.repeat_tx_start = xtrxllpciev0_repeat_tx_start,

	.get_sensor = xtrxllpciev0_get_sensor,
};

const struct xtrxll_ops* xtrxllpciev0_init(unsigned abi_version)
{
	if (XTRXLL_ABI_VERSION == abi_version) {
		return &mod_ops;
	}

	return NULL;
}

// Only when dynamic plugin is on
const struct xtrxll_ops* xtrxll_init(unsigned abi_version)
{
	return xtrxllpciev0_init(abi_version);
}
