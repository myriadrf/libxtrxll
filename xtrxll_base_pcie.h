/*
 * Internal base pcie header file
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
#ifndef XTRXLL_BASE_PCIE_H
#define XTRXLL_BASE_PCIE_H

#include <stdbool.h>
#include "xtrxll_base.h"


/* XTRX DMA control */
enum rxdma_stat {
	RXDMA_OVF        = 31,
	RXDMA_RE         = 30,
	RXDMA_BLK_REM    = 18,
	RXDMA_BLK_READY  = 17,
	RXDMA_FERESET    = 16,
	RXDMA_FEMODE     = 14,
	RXDMA_FEFMT      = 12,
	RXDMA_BUFNO_READ = 6,
	RXDMA_BUFNO      = 0,
};


enum txdma_stat {
	/** TxDMA buffer currently requesting by the DMA core  */
	TXDMA_BUFNO_TRANS_OFF = 8,

	/** TxDMA buffer that has been transfered into the air */
	TXDMA_BUFNO_CLEARED   = 16,

	/** TxDMA buffer ready for filling */
	TXDMA_BUFNO_WR        = 24,
};


struct xtrxll_base_pcie_dma {
	struct xtrxll_base_dev base;

	unsigned cfg_rx_bufsize;
	unsigned cfg_rx_desired_bufsize;
	unsigned tx_prev_burst_samples;

	//RX states
	int rx_rdsafe; ///< Number of buffers available for safe reading
	unsigned rd_block_samples; ///< Number of samples in the block (MIMO or SISO)
	unsigned rd_buf_idx;
	wts_long_t rd_cur_sample;  ///< Current timestamp (for all channels)

	unsigned tx_written;
	int tx_wrsafe;

	int tx_late_bursts;

	bool rx_owf_detected;
};

enum {
	XTRXLL_TXBUF_SIZE_MIMO_SYMBS = 2048,
};

enum pci_ids {
	XTRX_DID_V0 = 0x7012,
	XTRX_VID_V0 = 0x10ee,
};

enum pciebase_dmarx_flags {
	PCIEDMARX_FORCE_LOG = 1,
	PCIEDMARX_NO_CNTR_UPD = 2,
	PCIEDMARX_NO_CNTR_CHECK = 4,
};

int xtrxllpciebase_init(struct xtrxll_base_pcie_dma* dev);

int xtrxllpciebase_dmarx_resume(struct xtrxll_base_pcie_dma* dev, int chan,
								wts_long_t nxt);

int xtrxllpciebase_dmarx_get(struct xtrxll_base_pcie_dma* dev, int chan,
							 unsigned *pbufno, wts_long_t *wts, unsigned *sz,
							 unsigned flags, unsigned icnt);

int xtrxllpciebase_dmatx_post(struct xtrxll_base_pcie_dma* dev, int chan,
							  unsigned bufno, wts_long_t wts, uint32_t samples);

int xtrxllpciebase_dmatx_get(struct xtrxll_base_pcie_dma* dev, int chan,
							 unsigned* bufno, int* late, bool diag);

int xtrxllpciebase_repeat_tx(struct xtrxll_base_pcie_dma* dev,
							 int chan, xtrxll_fe_t fmt,
							 unsigned buf_szs, xtrxll_mode_t mode);

int xtrxllpciebase_repeat_tx_start(struct xtrxll_base_pcie_dma* dev,
								   int chan, int start);

int xtrxllpciebase_dma_start(struct xtrxll_base_pcie_dma* dev, int chan,
							 xtrxll_fe_t rxfe, xtrxll_mode_t rxmode,
							 wts_long_t rx_start_sample,
							 xtrxll_fe_t txfe, xtrxll_mode_t txmode);

int xtrxllpciebase_dmarx_stat(struct xtrxll_base_pcie_dma* dev);

#endif //XTRXLL_BASE_PCIE_H
