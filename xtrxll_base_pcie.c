/*
 * PCIe base source file
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
#include <inttypes.h>
#include <errno.h>
#include <stdlib.h>
#include <assert.h>

#include "xtrxll_base_pcie.h"
#include "xtrxll_log.h"

/* Include verilog file used for FPGA build */
#define localparam static const unsigned
#include "xtrxll_regs.vh"


int xtrxllpciebase_init(struct xtrxll_base_pcie_dma* dev)
{
	dev->cfg_rx_bufsize = RXDMA_MMAP_BUFF;
	dev->tx_prev_burst_samples = 0;

	dev->rx_rdsafe = 0;
	dev->rd_buf_idx = 0;
	dev->rd_cur_sample = 0;
	dev->rd_block_samples = 0;

	dev->tx_written = 0;
	dev->tx_wrsafe = 0;
	dev->tx_late_bursts = 0;

	dev->rx_owf_detected = false;
	return 0;
}

int xtrxllpciebase_dmarx_stat(struct xtrxll_base_pcie_dma* dev)
{
#if 1
	int res;
	uint32_t miss, odd, period;
	res = dev->base.selfops->reg_in(dev->base.self,
									UL_GP_ADDR + GP_PORT_RD_RXIQ_MISS,
									&miss);
	res = dev->base.selfops->reg_in(dev->base.self,
									UL_GP_ADDR + GP_PORT_RD_RXIQ_ODD,
									&odd);
	res = dev->base.selfops->reg_in(dev->base.self,
									UL_GP_ADDR + GP_PORT_RD_RXIQ_PERIOD,
									&period);
	XTRXLL_LOG(XTRXLL_ERROR, "XTRX P: %08x %08x <-> %08x\n", miss, odd, period);
	return res;
#else
	return 0;
#endif
}

int xtrxllpciebase_dmarx_get(struct xtrxll_base_pcie_dma* dev, int chan,
							 unsigned *pbufno, wts_long_t *wts, unsigned *sz,
							 unsigned flags, unsigned icnt)
{
	if (chan != 0)
		return -EINVAL;

	bool force_log = (flags & PCIEDMARX_FORCE_LOG);
	bool no_upd = (flags & PCIEDMARX_NO_CNTR_UPD);
	bool no_chk = (flags & PCIEDMARX_NO_CNTR_CHECK);

	unsigned bufno, bufno_rd;
	int res;
	if (dev->rx_rdsafe <= 0 || force_log) {
		uint32_t bufstat;
		res = dev->base.selfops->reg_in(dev->base.self,
										UL_GP_ADDR + GP_PORT_RD_RXDMA_STAT,
										&bufstat);
		if (res)
			return res;

		bufno    = (bufstat >> RXDMA_BUFNO) & 0x3f;
		bufno_rd = (bufstat >> RXDMA_BUFNO_READ) & 0x3f;
		unsigned blk_rem  = (bufstat >> RXDMA_BLK_REM) & 0xfff;

		XTRXLL_LOG((bufstat & (1U << RXDMA_OVF)) ? XTRXLL_ERROR : ((force_log) ? XTRXLL_INFO : XTRXLL_DEBUG),
				   "XTRX %s: RX DMA STAT %c%c %04dQW %c%c%d%d %02d/%02d I:%d\n",
				   dev->base.id,
				   (bufstat & (1U << RXDMA_OVF)) ? 'O': '-',
				   (bufstat & (1 << RXDMA_RE)) ?  'E': '-',
				   blk_rem,
				   (bufstat & (1 << RXDMA_BLK_READY)) ?  'Y': '-',
				   (bufstat & (1 << RXDMA_FERESET)) ?  'R': '-',
				   (bufstat >> RXDMA_FEMODE) & 3,
				   (bufstat >> RXDMA_FEFMT) & 3,
				   bufno_rd,
				   bufno,
				   icnt);

		//xtrxllpciebase_dmarx_stat(dev);
		if (bufstat & (1U << RXDMA_OVF)) {
			dev->rx_owf_detected = true;
			dev->rx_rdsafe = 0;
		}

		if (dev->rx_owf_detected && (bufno == bufno_rd || no_upd || no_chk)) {
			dev->rx_rdsafe = 0;
			dev->rx_owf_detected = false;

			//FIXME handle overflow flag
			wts32_t ofw_wts;
			res = dev->base.selfops->reg_in(dev->base.self,
											UL_GP_ADDR + GP_PORT_RD_RXDMA_STATTS,
											&ofw_wts);
			if (res)
				return res;

			uint32_t blk_timeoff = dev->rd_block_samples;
			wts32_t nxt     = ofw_wts - (ofw_wts % blk_timeoff) +
					((ofw_wts % blk_timeoff > blk_timeoff / 2) ?
						 2 * blk_timeoff : blk_timeoff);

			if (bufstat == ~0U && ofw_wts == ~0U) {
				// TODO: Most likely device is disconnected
				return -ENODEV;
			}

			wts_long_t nxt_high = (dev->rd_cur_sample & ~((wts_long_t)TS_WTS_INTERNAL_MASK)) | (nxt & TS_WTS_INTERNAL_MASK);
			if ((dev->rd_cur_sample & TS_WTS_INTERNAL_MASK) > (nxt & TS_WTS_INTERNAL_MASK))
				nxt_high += TS_WTS_INTERNAL_MASK + 1;

			// FIXME
			nxt_high += 256 * dev->rd_block_samples;

			XTRXLL_LOG(XTRXLL_INFO, "XTRX %s: BUF_OVF TS:%" PRIu64
					   " WTS:%d WTS_NXT:%d TS_NXT:%" PRIu64 " SKIP %" PRIu64 " buffers INT_S:%u\n",
					   dev->base.id, dev->rd_cur_sample, ofw_wts, nxt, nxt_high,
					   (nxt_high - dev->rd_cur_sample) / dev->rd_block_samples,
					   icnt);

			*wts = nxt_high;
			return -EOVERFLOW;
		} else if (!no_chk) {
			dev->rx_rdsafe = ((bufno - bufno_rd) & 0x3f);
			dev->rx_rdsafe--;

			/* WR pointer can be only RXDMA_BUFFERS buffers ahead */
			if (((bufno - bufno_rd) & 0x3f) > RXDMA_BUFFERS) {
				XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Incorrect DMA pointers! (bufno=%d bufno_rd=%d rdidx=%d icnt=%d)\n",
						   dev->base.id, bufno, bufno_rd, dev->rd_buf_idx, icnt);
				return -EPIPE;
			}

			if (bufno == bufno_rd) {
				return -EAGAIN;
			}
		} else {
			return -EAGAIN;
		}
	} else {
		XTRXLL_LOG(XTRXLL_DEBUG, "XTRX %s: RD %d of %d\n",
				   dev->base.id, dev->rd_buf_idx, dev->rx_rdsafe);
		dev->rx_rdsafe--;
		bufno_rd = dev->rd_buf_idx;
	}

	/* we've got a valid RX buffer */
	if (wts) {
		*wts = dev->rd_cur_sample;
	}

	if (!no_upd) {
		// update counters TODO move away from here
		if (wts) {
			dev->rd_cur_sample += dev->rd_block_samples;
		}

		if (dev->rd_buf_idx != bufno_rd)
			abort();

		dev->rd_buf_idx = (dev->rd_buf_idx + 1) & 0x3f;
	}

	*pbufno = bufno_rd & 0x1f;
	*sz = dev->cfg_rx_bufsize;
	return 0;
}


int xtrxllpciebase_dmarx_resume(struct xtrxll_base_pcie_dma* dev, int chan,
								wts_long_t nxt)
{
	if (chan != 0)
		return -EINVAL;

	/* Issue timed command to resume rx */
	//xtrxll_issue_timmed_command(dev, nxt, TC_ROUTE_RXFE, 0);
	int res = dev->base.ctrlops->issue_timmed_command(&dev->base, nxt,
													  TC_ROUTE_RXFE, 0);
	if (res)
		return res;

	dev->rd_cur_sample = nxt;
	return 0;
}


int xtrxllpciebase_dmatx_post(struct xtrxll_base_pcie_dma* dev, int chan,
							  unsigned bufno, wts_long_t wts, uint32_t samples)
{
	int res;
	if (chan != 0)
		return -EINVAL;
	if (samples > 4096)
		return -EINVAL;

	XTRXLL_LOG(XTRXLL_DEBUG, "XTRX %s: TX DMA POST %u TS %" PRIu64 " SAMPLES %u\n",
			   dev->base.id, bufno, wts, samples);

	if (bufno > 0x1f)
		return -EINVAL;

	if (dev->tx_prev_burst_samples != samples) {
		res = dev->base.selfops->reg_out(dev->base.self,
										 UL_GP_ADDR + GP_PORT_WR_TXDMA_CNF_L,
										 samples);
		if (res)
			return res;

		dev->tx_prev_burst_samples = samples;
	}

	return dev->base.selfops->reg_out(dev->base.self,
									  UL_GP_ADDR + GP_PORT_WR_TXDMA_CNF_T, wts);
}

int xtrxllpciebase_dmatx_get(struct xtrxll_base_pcie_dma* dev, int chan,
							 unsigned* bufno, int* late, bool diag)
{
	if (chan != 0)
		return -EINVAL;

	unsigned nwr;
	if (dev->tx_wrsafe <= 0 || bufno == NULL || s_loglevel >= XTRXLL_PARANOIC) {
		enum {
			IDX_STAT = 0,
			IDX_STATM = 1,
			IDX_STATTS = 2,
			IDX_ST_CPL = 3,
			TOTAL = 4
		};

		/* check that we didn't screw the register placement */
		assert(GP_PORT_RD_TXDMA_STAT + IDX_STATM == GP_PORT_RD_TXDMA_STATM);
		assert(GP_PORT_RD_TXDMA_STAT + IDX_STATTS == GP_PORT_RD_TXDMA_STATTS);
		assert(GP_PORT_RD_TXDMA_STAT + IDX_ST_CPL == GP_PORT_RD_TXDMA_ST_CPL);

		uint32_t stat_regs[TOTAL] = { ~0U, ~0U, ~0U, ~0U };
		int res;
		unsigned cnt = (s_loglevel >= XTRXLL_DEBUG || (bufno == NULL) || diag) ? 4 :
					   (late) ? 2 : 1;
		res = dev->base.selfops->reg_in_n(dev->base.self,
										  UL_GP_ADDR + GP_PORT_RD_TXDMA_STAT,
										  stat_regs,
										  cnt);
		if (res)
			return res;

		uint32_t bufstat = stat_regs[IDX_STAT];
		uint32_t statm = stat_regs[IDX_STATM];
		uint32_t ts = stat_regs[IDX_STATTS];
		uint32_t cpls = stat_regs[IDX_ST_CPL];

		unsigned ntrans   = (bufstat >> TXDMA_BUFNO_TRANS_OFF) & 0x3f;
		unsigned ncleared = (bufstat >> TXDMA_BUFNO_CLEARED) & 0x3f;
		unsigned rdx      = (((bufstat >> TXDMA_BUFNO_TRANS_OFF) & 0xc0) >> 6) |
				(((bufstat >> TXDMA_BUFNO_CLEARED) & 0xc0) >> 4) |
				(((bufstat >> TXDMA_BUFNO_WR) & 0xc0) >> 2);

		unsigned dma_stat = bufstat & 0xff;
		nwr      = (bufstat >> TXDMA_BUFNO_WR) & 0x3f;


		XTRXLL_LOG((bufno == NULL || diag) ? XTRXLL_WARNING : XTRXLL_DEBUG,
				   "XTRX %s: TX DMA STAT %02d|%02d/%02d/%02d/%02d RESET:%d "
				   "Full:%d TxS:%x  %02d/%02d FE:%d FLY:%x D:%d TS:%d CPL:%08x\n",
				   dev->base.id, dev->tx_written, nwr, ncleared, ntrans, rdx,
				   (dma_stat & 0x80) ? 1 : 0,
				   ((dma_stat >> 3) & 0xf),
				   ((dma_stat) & 7),
				   statm & 0x3f,
				   (statm >> 6) & 0x3f,
				   (statm >> 12) & 0x3,
				   (statm >> 14) & 0x3,
				   (statm >> 16),
				   ts, cpls);

		if (((nwr - ncleared) & 0x3f) > TXDMA_BUFFERS) {
			abort();
			return -EPIPE;
		}
		//if (((dev->tx_written - ncleared) & 0x3f) >= TXDMA_BUFFERS - 2)
		//	return -EBUSY;
		if (((dev->tx_written - ncleared) & 0x3f) >= TXDMA_BUFFERS - 1)
			return -EBUSY;

		nwr = dev->tx_written;

		if (bufno) {
			dev->tx_written = (dev->tx_written + 1) & 0x3f;
		}
		dev->tx_wrsafe = TXDMA_BUFFERS - 2 - ((dev->tx_written - ncleared) & 0x3f);
		if (late) {
			dev->tx_late_bursts = (statm >> 16);
		}
	} else {
		nwr = dev->tx_written;
		dev->tx_written = (dev->tx_written + 1) & 0x3f;
		--dev->tx_wrsafe;

		XTRXLL_LOG(XTRXLL_DEBUG, "XTRX %s: TX DMA CACHE  %02d (free:%02d)\n",
				   dev->base.id, nwr, dev->tx_wrsafe);
	}

	if (late) {
		*late = dev->tx_late_bursts;
	}
	if (bufno) {
		*bufno = nwr & 0x1f;
	}
	return 0;
}

int xtrxllpciebase_repeat_tx(struct xtrxll_base_pcie_dma* dev,
							 int chan, xtrxll_fe_t fmt,
							 unsigned buf_szs, xtrxll_mode_t mode)
{
	int res;
	if (chan !=0)
		return -EINVAL;
	if (fmt != XTRXLL_FE_16BIT)
		return -EINVAL;

	// 2*2*2 == 2 (MIMO-> 2ch) * 2 (I/Q) * 2 (16bits)
	if (buf_szs > XTRXLL_TXBUF_SIZE_MIMO_SYMBS * 2 * 2 * 2)
		buf_szs = XTRXLL_TXBUF_SIZE_MIMO_SYMBS * 2 * 2 * 2;

	// repeat_sm in IQ samles
	unsigned repeat_sm = buf_szs >> 3;
	int mode_siso = (mode & XTRXLL_FE_MODE_SISO);
	uint32_t reg = (1UL << GP_PORT_WR_RXTXDMA_TXV)| fmt |
			(1 << GP_PORT_TXDMA_CTRL_MODE_REP) |
			(mode_siso ? (1 << GP_PORT_TXDMA_CTRL_MODE_SISO) : 0) |
			(0 << GP_PORT_TXDMA_CTRL_MODE_INTER_OFF); // |
	//((repeat_sm - (mode_siso ? 1 : 2)) << GP_PORT_TXDMA_CTRL_REP_OFF);

	res = dev->base.selfops->reg_out(dev->base.self,
									 UL_GP_ADDR + GP_PORT_WR_RXTXDMA, reg);
	if (res)
		return res;

	res = dev->base.selfops->reg_out(dev->base.self,
									 UL_GP_ADDR + GP_PORT_WR_TXDMA_CNF_L,
									 repeat_sm);
	if (res)
		return res;

	res = dev->base.selfops->reg_out(dev->base.self,
									 UL_GP_ADDR + GP_PORT_WR_TXDMA_CNF_T, 0);
	if (res)
		return res;

	uint32_t st;
	res = dev->base.selfops->reg_in(dev->base.self, GP_PORT_RD_TXDMA_STAT,
										  &st);
	if (res)
		return res;

	XTRXLL_LOG(XTRXLL_INFO,  "XTRX %s: REPEAT TS %s %c - %d =>%08x\n",
			   dev->base.id,
			   (fmt == XTRXLL_FE_STOP)  ? "STOP" :
			   (fmt == XTRXLL_FE_8BIT)  ? "8 bit" :
			   (fmt == XTRXLL_FE_12BIT) ? "12 bit" :
			   (fmt == XTRXLL_FE_16BIT) ? "16 bit" : "<unkn>",
			   ((mode & XTRXLL_FE_MODE_SISO)   ? 'S' : '-'),
			   repeat_sm, st);
	return 0;
}

int xtrxllpciebase_repeat_tx_start(struct xtrxll_base_pcie_dma* dev,
								   int chan, int start)
{
	int res;
	if (chan != 0)
		return -EINVAL;

	res = dev->base.selfops->reg_out(dev->base.self,
									 UL_GP_ADDR + GP_PORT_WR_RXTXDMA,
									 (1UL << GP_PORT_WR_RXTXDMA_TXV) |
									 (1 << GP_PORT_TXDMA_CTRL_MODE_REP) |
									 ((start) ? XTRXLL_FE_16BIT : XTRXLL_FE_STOP));
	if (res)
		return res;

	uint32_t st;
	res = dev->base.selfops->reg_in(dev->base.self, GP_PORT_RD_TXDMA_STAT, &st);
	if (res)
		return res;

	XTRXLL_LOG(XTRXLL_INFO,  "XTRX %s: REPEAT %s =>%08x\n",
			   dev->base.id, (start) ? "START" : "STOP", st);
	return 0;
}

static unsigned get_max_samples_in_buffer(xtrxll_fe_t fe, xtrxll_mode_t mode,
										  unsigned bufsize)
{
	if (mode == XTRXLL_FE_MODE_MIMO)
		bufsize >>= 1;

	switch (fe) {
	case XTRXLL_FE_8BIT:  return bufsize >> 1;
	case XTRXLL_FE_12BIT: return (bufsize * 3) >> 2;
	case XTRXLL_FE_16BIT: return bufsize >> 2;
	default: break;
	}

	return 0;
}

int xtrxllpciebase_dma_start(struct xtrxll_base_pcie_dma* dev, int chan,
							 xtrxll_fe_t rxfe, xtrxll_mode_t rxmode,
							 wts_long_t rx_start_sample,
							 xtrxll_fe_t txfe, xtrxll_mode_t txmode)
{
	int res;
	if (chan != 0)
		return -EINVAL;

	if (rxfe != XTRXLL_FE_DONTTOUCH && (rxfe & ~3))
		return -EINVAL;

	if (txfe != XTRXLL_FE_DONTTOUCH && (txfe & ~3))
		return -EINVAL;

	if (rxfe == XTRXLL_FE_DONTTOUCH && txfe == XTRXLL_FE_DONTTOUCH)
		return -EINVAL;

	if (rx_start_sample > TS_WTS_INTERNAL_MASK)
		return -EINVAL;

	uint32_t reg = 0;

	if (rxfe != XTRXLL_FE_DONTTOUCH) {
		dev->rx_owf_detected = false;
		dev->rx_rdsafe = 0;
		dev->rd_buf_idx = 0;
		dev->rd_cur_sample = rx_start_sample;
		dev->rd_block_samples = get_max_samples_in_buffer(rxfe, rxmode, dev->cfg_rx_bufsize);
		if (rx_start_sample) {
			reg |= (1UL << (WR_RXDMA_FE_PAUSED + GP_PORT_WR_RXTXDMA_RXOFF));
		}

		switch (rxmode & XTRXLL_FE_MODE_OVER_MASK) {
		case XTRXLL_FE_MODE_4X_ACC_OVER:
			reg |= 1UL << (WR_RXDMA_FE_DECIM_OFF + GP_PORT_WR_RXTXDMA_RXOFF);
			break;
		case XTRXLL_FE_MODE_16X_ACC_OVER:
			reg |= 2UL << (WR_RXDMA_FE_DECIM_OFF + GP_PORT_WR_RXTXDMA_RXOFF);
			break;
		case XTRXLL_FE_MODE_16X_NO_ACC:
			reg |= 3UL << (WR_RXDMA_FE_DECIM_OFF + GP_PORT_WR_RXTXDMA_RXOFF);
			break;
		default: break;
		}

		if ((rxmode & XTRXLL_FE_MODE_SISO)) {
			reg |= 1UL << (WR_RXDMA_FE_SISO + GP_PORT_WR_RXTXDMA_RXOFF);
		}

		reg |= (1UL << GP_PORT_WR_RXTXDMA_RXV) | ((uint32_t)rxfe << GP_PORT_WR_RXTXDMA_RXOFF);
	}

	if (txfe != XTRXLL_FE_DONTTOUCH) {
		dev->tx_late_bursts = 0;
		dev->tx_wrsafe = -1;
		dev->tx_written = 0;

		reg |= (1UL << GP_PORT_WR_RXTXDMA_TXV)
				| (txfe)
				| ((txmode & XTRXLL_FE_MODE_SISO) ? (1 << GP_PORT_TXDMA_CTRL_MODE_SISO) : 0)
				| ((((uint32_t)txmode >> XTRXLL_FE_MODE_INTER_OFF) & XTRXLL_FE_MODE_INTER_MASK) << GP_PORT_TXDMA_CTRL_MODE_INTER_OFF);
	}

	if (rxfe == XTRXLL_FE_STOP) {
		xtrxllpciebase_dmarx_stat(dev);
		xtrxllpciebase_dmarx_stat(dev);
	}

	XTRXLL_LOG(XTRXLL_INFO,  "XTRX %s: RX DMA %s %s (BLK:%d TS:%" PRIu64 "); TX DMA %s %s\n", dev->base.id,
			   (rxfe == XTRXLL_FE_DONTTOUCH) ? "SKIP" :
			   (rxfe == XTRXLL_FE_STOP)  ? "STOP" :
			   (rxfe == XTRXLL_FE_8BIT)  ? "8 bit" :
			   (rxfe == XTRXLL_FE_12BIT) ? "12 bit" :
			   (rxfe == XTRXLL_FE_16BIT) ? "16 bit" : "<unkn>",
			   ((rxmode & XTRXLL_FE_MODE_SISO)   ? "SISO" : "MIMO"),
			   dev->rd_block_samples, rx_start_sample,
			   (txfe == XTRXLL_FE_DONTTOUCH) ? "SKIP" :
			   (txfe == XTRXLL_FE_STOP)  ? "STOP" :
			   (txfe == XTRXLL_FE_8BIT)  ? "8 bit" :
			   (txfe == XTRXLL_FE_12BIT) ? "12 bit" :
			   (txfe == XTRXLL_FE_16BIT) ? "16 bit" : "<unkn>",
			   ((txmode & XTRXLL_FE_MODE_SISO)   ? "SISO" : "MIMO"));

	res = dev->base.selfops->reg_out(dev->base.self, UL_GP_ADDR + GP_PORT_WR_RXTXDMA, reg);
	if (res)
		return res;

	if (rxfe != XTRXLL_FE_DONTTOUCH && rxfe != XTRXLL_FE_STOP && rx_start_sample) {
		xtrxllpciebase_dmarx_resume(dev, chan, rx_start_sample);
	}

	/* Put RX FE in reset state when we stop transmition. We need to do it
   * separately due to implimentation */
	if (rxfe == XTRXLL_FE_STOP) {
		res = dev->base.selfops->reg_out(dev->base.self, UL_GP_ADDR + GP_PORT_WR_RXTXDMA,
								(1UL << GP_PORT_WR_RXTXDMA_RXV) |
								(1UL << (WR_RXDMA_FE_RESET + GP_PORT_WR_RXTXDMA_RXOFF)));
		if (res)
			return res;
	}
	return 0;
}



