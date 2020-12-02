/*
 * Public xtrx low level base header file
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
#ifndef XTRXLL_BASE_H
#define XTRXLL_BASE_H

#include "xtrxll_api.h"

struct xtrxll_ops;
struct xtrxll_ctrl_ops;

#define XTRXLL_INIT_FUNC "xtrxll_init"

typedef const struct xtrxll_ops* (*xtrxll_init_func_t)(unsigned abi_version);


#define XTRXLL_ABI_VERSION 0x1000001

#define GET_HWID_COMPAT(x) ((x >> 8) & 0xff)
/**
 * @brief The xtrxll_base_dev struct
 *
 * Basic structure that every xtrxll object starts with
 */
#define XTRXLL_INTDATA_STORAGE_ROOM 7

struct xtrxll_base_dev {
	const struct xtrxll_ops* selfops;
	const struct xtrxll_ctrl_ops* ctrlops;

	struct xtrxll_base_dev* self; ///< Pointer to the original device
	const char* id;

	uint32_t hwid;
	uint32_t internal_state[XTRXLL_INTDATA_STORAGE_ROOM]; ///< internal state (DO NOT TOUCH)
};

int xtrxll_base_dev_init(struct xtrxll_base_dev* dev,
						  const struct xtrxll_ops *ops,
						  const char* id);

struct xtrxll_ops {
	int (*open)(const char* device, unsigned flags, struct xtrxll_base_dev** dev);
	void (*close)(struct xtrxll_base_dev* dev);
	int (*discovery)(xtrxll_device_info_t* buffer, size_t maxbuf);
	const char* (*get_proto_id)(void);

	int (*reg_out)(struct xtrxll_base_dev* dev, unsigned reg, uint32_t outval);
	int (*reg_in)(struct xtrxll_base_dev* dev, unsigned reg, uint32_t* inval);

	int (*reg_out_n)(struct xtrxll_base_dev* dev, unsigned streg,
					 const uint32_t* outval, unsigned count);
	int (*reg_in_n)(struct xtrxll_base_dev* dev, unsigned streg,
					uint32_t* inval, unsigned count);

	int (*spi_bulk)(struct xtrxll_base_dev* dev, uint32_t lmsno,
					const uint32_t* out, uint32_t* in, size_t count);
	int (*i2c_cmd)(struct xtrxll_base_dev* dev, uint32_t cmd, uint32_t *dout);

	// RX DMA
	int (*dma_rx_init)(struct xtrxll_base_dev* dev, int chan, unsigned buf_szs,
					   unsigned* out_szs);
	int (*dma_rx_deinit)(struct xtrxll_base_dev* dev, int chan);

	int (*dma_rx_getnext)(struct xtrxll_base_dev* dev, int chan, void** addr,
						  wts_long_t *ts, unsigned *sz, unsigned flags,
						  unsigned timeout_ms);
	int (*dma_rx_release)(struct xtrxll_base_dev* dev, int chan, void* addr);

	int (*dma_rx_resume_at)(struct xtrxll_base_dev* dev, int chan, wts_long_t nxt);

	// TX DMA
	int (*dma_tx_init)(struct xtrxll_base_dev* dev, int chan, unsigned buf_szs);
	int (*dma_tx_deinit)(struct xtrxll_base_dev* dev, int chan);

	int (*dma_tx_getfree_ex)(struct xtrxll_base_dev* dev, int chan, void** addr,
							 uint16_t* late, unsigned timeout_ms);
	int (*dma_tx_post)(struct xtrxll_base_dev* dev, int chan, void* addr,
					   wts_long_t wts, uint32_t samples);

	int (*dma_start)(struct xtrxll_base_dev* dev, int chan,
					 const struct xtrxll_dmaop* op);

	int (*repeat_tx_buf)(struct xtrxll_base_dev* dev, int chan, xtrxll_fe_t fmt,
						 const void* buff, unsigned buf_szs, xtrxll_mode_t mode);
	int (*repeat_tx_start)(struct xtrxll_base_dev* dev, int chan, int start);


	// Sensor wrapper with wait functions
	int (*get_sensor)(struct xtrxll_base_dev* dev, unsigned sensorno, int* outval);
	int (*set_param)(struct xtrxll_base_dev* dev, unsigned paramno, uintptr_t inval);
};

/**
 * @brief wts32_t Wrappable timestamp number for TX/RX samples
 */
typedef uint32_t wts32_t;

enum {
	TC_ROUTE_RXFE = 0,
};

#define TS_WTS_INTERNAL_MASK  ((1 << TC_TS_BITS) - 1)

enum drp_acc_type {
	DRP_SET_REG_WR = 0,
	DRP_SET_REG_RD = 1,
	DRP_SET_GPIO = 2,
};

enum {
	MAX_DRPS = 4,
};

struct xtrxll_ctrl_ops {
	int (*get_sensor)(struct xtrxll_base_dev* dev, unsigned sensorno,
					  int* outval);

	int (*drp_set)(struct xtrxll_base_dev* dev, unsigned drpno,
				   uint16_t reg, uint16_t value,
				   unsigned drp_gpio, unsigned acc_type);
	int (*drp_get)(struct xtrxll_base_dev* dev, unsigned drpno,
				   uint16_t *reg_value, unsigned* drp_gpio);


	int (*issue_timmed_command)(struct xtrxll_base_dev* dev, wts32_t time,
								unsigned route,	uint32_t data);

	int (*read_uart)(struct xtrxll_base_dev* dev, unsigned uartno, uint8_t* out,
					 unsigned maxsize, unsigned *written);

	// READ & WRITE to XTRX RAM
	int (*mem_rb32)(struct xtrxll_base_dev* dev, uint32_t xtrx_addr,
					unsigned mwords, uint32_t* host_addr);
	int (*mem_wr32)(struct xtrxll_base_dev* dev, uint32_t xtrx_addr,
					unsigned mwords, const uint32_t* host_addr);

	int (*set_param)(struct xtrxll_base_dev* dev, unsigned paramno, uintptr_t inval);
};

#endif //XTRXLL_BASE_H

