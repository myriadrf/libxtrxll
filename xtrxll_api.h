/*
 * Public xtrx low level API header file
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
#ifndef XTRXLL_API_H
#define XTRXLL_API_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XTRXLL_API

struct xtrxll_dev;

enum xtrxll_claim_flags {
	XTRXLL_CLAIM_UART_SIM = 0x0001,
	XTRXLL_CLAIM_UART_GPS = 0x0002,
	XTRXLL_CLAIM_HWMON    = 0x0004,
	XTRXLL_CLAIM_GPSDO    = 0x0008,

	XTRXLL_CLAIM_TX       = 0x0010,
	XTRXLL_CLAIM_RX       = 0x0020,

	XTRXLL_FULL_DEV_MATCH = 0x0040,

	// Lock transaivers on the same board
	XTRXLL_CLAIM_DEV_MSK0 = 0x1000,
	XTRXLL_CLAIM_DEV_MSK1 = 0x2000,
	XTRXLL_CLAIM_DEV_MSK2 = 0x4000,
	XTRXLL_CLAIM_DEV_MSK3 = 0x8000,
};

typedef enum product_type {
	PRODUCT_XTRX = 0,
} product_type_t;

enum {
	DEV_UNIQNAME_MAX = 64,
	DEV_ADDR_MAX = 16,
	DEV_PROTO_MAX = 16,
	DEV_BUSSPEED_MAX = 16,
};

typedef struct xtrxll_device_info {
	char uniqname[DEV_UNIQNAME_MAX];
	char proto[DEV_PROTO_MAX];
	char addr[DEV_ADDR_MAX];
	char busspeed[DEV_BUSSPEED_MAX];
	uint32_t product_id;
	uint32_t revision;
} xtrxll_device_info_t;

/** Open xtrx device
 *
 */
XTRXLL_API int xtrxll_open(const char* device, unsigned flags,
						   struct xtrxll_dev** dev);
XTRXLL_API void xtrxll_close(struct xtrxll_dev* dev);

/** Discovery all low-level devices on every plugin */
XTRXLL_API int xtrxll_discovery(xtrxll_device_info_t* buffer,
								size_t maxbuf);

XTRXLL_API const char* xtrxll_get_name(struct xtrxll_dev* dev);

enum xtrxll_lms7_mask {
	XTRXLL_LMS7_0 = 1,
	XTRXLL_LMS7_1 = 2,
	XTRXLL_LMS7_ALL = -1,
};

/** LMS7 SPI
 *
 */
XTRXLL_API int xtrxll_lms7_spi_bulk(struct xtrxll_dev* dev, uint32_t lmsno,
									const uint32_t* out, uint32_t* in,
									size_t count);

enum xtrxll_sensors {
	XTRXLL_TEMP_SENSOR_CUR,
	XTRXLL_TEMP_SENSOR_MAX,
	XTRXLL_TEMP_SENSOR_MIN,
	XTRXLL_OSC_LATCHED,
	XTRXLL_ONEPPS_CAPTURED,
	XTRXLL_REFCLK_CNTR,
	XTRXLL_REFCLK_CLK,
	XTRXLL_DAC_REG,

	XTRXLL_GTIME_SECFRAC,
	XTRXLL_GTIME_OFF,

	XTRXLL_GPIO_IN,

	XTRXLL_TX_TIME,
	XTRXLL_RX_TIME,

	XTRXLL_PMIC0_VER,
	XTRXLL_PMIC0_ID,
	XTRXLL_PMIC0_CTRL1,
	XTRXLL_PMIC1_VER,
	XTRXLL_PMIC1_ID,
	XTRXLL_PMIC1_CTRL1,

	XTRXLL_DMABUF_RXST64K, /**< RX: 0 - 65535 buffer fullness */
	XTRXLL_DMABUF_TXST64K, /**< TX: 0 - 65535 buffer fullness */

	/* Special test sensors / not portable, for MGF & testing */
	XTRXLL_TEST_CNT_RXIQ_MISS,
	XTRXLL_TEST_CNT_RXIQ_MALGN,

	XTRXLL_HWID,

	XTRXLL_XTRX_VIO,
	XTRXLL_XTRX_VGPIO,
	XTRXLL_FE_CTRL,
	XTRXLL_EXT_CLK,

	XTRXLL_CFG_NUM_RFIC,
	XTRXLL_CFG_HAS_GPS,
	XTRXLL_CFG_HAS_SIM_READER,

	XTRXLL_EXT_SPI_RB,

};

XTRXLL_API int xtrxll_get_sensor(struct xtrxll_dev* dev, unsigned sensorno,
								 int* outval);

enum param_pwr_ctrl {
	PWR_CTRL_PDOWN = 0,
	PWR_CTRL_BUSONLY = 1,
	PWR_CTRL_ON = 2,
};

enum xtrxll_ppsdo_ctrl {
	XTRXLL_PPSDO_DISABLE = 0,
	XTRXLL_PPSDO_INT_GPS = 1,
	XTRXLL_PPSDO_EXT_PPS = 2,
};

enum xtrxll_gtime_ctrl {
	XTRXLL_GTIME_DISABLE   = 0,
	XTRXLL_GTIME_INT_ISO   = 1,
	XTRXLL_GTIME_EXT_PPS   = 2,
	XTRXLL_GTIME_EXT_PPSFW = 3,
};

enum xtrxll_giso_ctrl {
	XTRXLL_GISO_DISABLE = 0,
	XTRXLL_GISO_PPSFW = 1,
};

/**
 * @brief The xtrxll_clksel enum
 *
 * Values for @ref XTRXLL_PARAM_EXT_CLK
 */
enum xtrxll_clksel {
	XTRXLL_CLK_INT = 0,
	XTRXLL_CLK_INT_PD = 1,
	XTRXLL_CLK_EXT_NOPD = 2,
	XTRXLL_CLK_EXT = 3,
	XTRXLL_CLK_MASK = 3,
};

typedef enum xtrxll_params {
	XTRXLL_PARAM_CLOCK_TYPE,
	XTRXLL_PARAM_PWR_MODE,
	XTRXLL_PARAM_PWR_VIO,
	XTRXLL_PARAM_PWR_VGPIO,
	XTRXLL_PARAM_RX_DLY,
	XTRXLL_PARAM_REF_DAC,
	XTRXLL_PARAM_SWITCH_RX_ANT,
	XTRXLL_PARAM_SWITCH_TX_ANT,
	XTRXLL_PARAM_PWR_CTRL,
	XTRXLL_PARAM_FE_CTRL,
	XTRXLL_PARAM_EXT_CLK,
	XTRXLL_PARAM_DSPFE_CMD,

	XTRXLL_PARAM_GPIO_FUNC,
	XTRXLL_PARAM_GPIO_DIR,
	XTRXLL_PARAM_GPIO_OUT,
	XTRXLL_PARAM_GPIO_CS,

	XTRXLL_PARAM_PPSDO_CTRL,
	XTRXLL_PARAM_GTIME_CTRL,
	XTRXLL_PARAM_ISOPPS_CTRL,

	XTRXLL_PARAM_GTIME_SETCMP,
	XTRXLL_PARAM_GTIME_TRIMOFF,
	XTRXLL_PARAM_ISOPPS_SETTIME,

	XTRXLL_PARAM_GTIME_RESET,
	XTRXLL_PARAM_GTIME_LOAD_CMD,
	XTRXLL_PARAM_GTIME_LOAD_TIME,

	XTRXLL_PARAM_EXT_SPI,

	XTRXLL_PARAM_CURPPS_SETTIME,
} xtrxll_params_t;


typedef enum xtrxll_gtime_cmd_type {
	XTRXLL_GCMDT_RFIC_CMD,
	XTRXLL_GCMDT_TRX_CMD,
	XTRXLL_GCMDT_GPIO_SET,
	XTRXLL_GCMDT_GPIO_CS,

	// Get rid of
	XTRXLL_GCMDT_RXCMDT_CMD,
	XTRXLL_GCMDT_RXCMDD_CMD,
} xtrxll_gtime_cmd_type_t;

struct xtrxll_gtime_cmd {
	xtrxll_gtime_cmd_type_t type;
	unsigned cmd_idx;
	uint32_t param;
};

struct xtrxll_gtime_time {
	unsigned d_idx;
	unsigned d_cnt;
	uint32_t sec;
	uint32_t frac;
};

XTRXLL_API int xtrxll_set_param(struct xtrxll_dev* dev, unsigned paramno,
								uintptr_t value);

// Frone end treaming format
typedef enum xtrxll_fe {
	XTRXLL_FE_STOP = 0,
	XTRXLL_FE_8BIT = 1,
	XTRXLL_FE_12BIT = 2,
	XTRXLL_FE_16BIT = 3,

	XTRXLL_FE_DONTTOUCH = 255
} xtrxll_fe_t;

typedef uint64_t wts_long_t;

// RX DMA
XTRXLL_API int xtrxll_dma_rx_init(struct xtrxll_dev* dev, int chan,
								  unsigned buf_szs, unsigned *out_szs);
XTRXLL_API int xtrxll_dma_rx_deinit(struct xtrxll_dev* dev, int chan);

typedef enum xtrxll_dma_rx_flags {
	XTRXLL_RX_DONTWAIT = 1,

	/**< Do not stall on internal buffer overflow, just report the -EOVERFLOW
	 * with the specific timestamp. One of the next call will return actual buffer
	 * if it's available.
	 */
	XTRXLL_RX_NOSTALL = 2,

	XTRXLL_RX_FORCELOG = 4,

	XTRXLL_RX_SPURSINTLOG = 8,

	XTRXLL_RX_REPORT_TIMEOUT = 16,
} xtrxll_dma_rx_flags_t;

XTRXLL_API int xtrxll_dma_rx_getnext(struct xtrxll_dev* dev, int chan,
									 void** addr, wts_long_t *ts, unsigned *sz,
									 unsigned flags, unsigned timeout_ms);
XTRXLL_API int xtrxll_dma_rx_release(struct xtrxll_dev* dev, int chan,
									 void* addr);

XTRXLL_API int xtrxll_dma_rx_resume_at(struct xtrxll_dev* dev, int chan,
									   wts_long_t nxt);

// TX DMA
XTRXLL_API int xtrxll_dma_tx_init(struct xtrxll_dev* dev, int chan,
								  unsigned buf_szs);
XTRXLL_API int xtrxll_dma_tx_deinit(struct xtrxll_dev* dev, int chan);

XTRXLL_API int xtrxll_dma_tx_getfree_ex(struct xtrxll_dev* dev, int chan,
										void** addr, uint16_t* late,
										unsigned timeout_ms);

/**
 * @brief xtrxll_dma_tx_post
 * @param dev
 * @param chan
 * @param addr
 * @param wts
 * @param samples number of MIMO samples
 * @return
 */
XTRXLL_API int xtrxll_dma_tx_post(struct xtrxll_dev* dev, int chan, void* addr,
								  wts_long_t wts, uint32_t samples);

typedef enum xtrxll_mode {
	XTRXLL_FE_MODE_MIMO   = 0,
	XTRXLL_FE_MODE_SISO   = 1,
	XTRXLL_FE_MODE_SM_MSK = 1,

	XTRXLL_FE_MODE_RXDSP_BYPASS   = 0,
	XTRXLL_FE_MODE_RXDSP_MODE1    = 4,
	XTRXLL_FE_MODE_RXDSP_MODE2    = 8,
	XTRXLL_FE_MODE_RXDSP_MODE3    = 12,
	XTRXLL_FE_MODE_RXDSP_MASK     = 12,

	XTRXLL_FE_MODE_INTER_OFF  = 8,
	XTRXLL_FE_MODE_INTER_MASK = 7,

} xtrxll_mode_t;

struct xtrxll_dmaop {
	xtrxll_fe_t rxfe;
	xtrxll_mode_t rxmode;

	xtrxll_fe_t txfe;
	xtrxll_mode_t txmode;

	wts_long_t rx_start_sample;

	uint32_t gtime_sec;
	uint32_t gtime_frac;

	unsigned gidx;
};

XTRXLL_API int xtrxll_dma_start(struct xtrxll_dev* dev, int chan,
								const struct xtrxll_dmaop* op);

static inline int xtrxll_dma_rx_start(struct xtrxll_dev* dev, int chan,
									  xtrxll_fe_t fe)
{
	struct xtrxll_dmaop op;
	op.rxfe = fe;
	op.rxmode = XTRXLL_FE_MODE_MIMO;
	op.txfe = XTRXLL_FE_DONTTOUCH;
	op.txmode = XTRXLL_FE_MODE_MIMO;
	op.rx_start_sample = 0;

	op.gtime_sec = 0;
	op.gtime_frac = 0;

	return xtrxll_dma_start(dev, chan, &op);
}


static inline int xtrxll_dma_tx_start(struct xtrxll_dev* dev, int chan,
									  xtrxll_fe_t fe, xtrxll_mode_t mode)
{
	struct xtrxll_dmaop op;
	op.rxfe = XTRXLL_FE_DONTTOUCH;
	op.rxmode = XTRXLL_FE_MODE_MIMO;
	op.txfe = fe;
	op.txmode = mode;
	op.rx_start_sample = 0;

	op.gtime_sec = 0;
	op.gtime_frac = 0;

	return xtrxll_dma_start(dev, chan, &op);
}


/**
 * @brief xtrxll_fill_repeat_buf
 * @param dev
 * @param fmt
 * @param buff     Address of user buffer to copy
 * @param buf_szs
 * @return
 */
XTRXLL_API int xtrxll_repeat_tx_buf(struct xtrxll_dev* dev, int chan, xtrxll_fe_t fmt,
									const void* buff, unsigned buf_szs, xtrxll_mode_t mode);

/**
 * @brief xtrxll_repeat_tx_start
 * @param dev
 * @param start
 * @return
 *
 * After calling xtrxll_repeat_tx_start with @param start == 0 xtrxll_repeat_tx_buf should be
 * called again before calling with @param start == 1
 */
XTRXLL_API int xtrxll_repeat_tx_start(struct xtrxll_dev* dev, int chan, int start);


XTRXLL_API int xtrxll_read_uart(struct xtrxll_dev* dev, unsigned uartno,
								uint8_t* out, unsigned maxsize, unsigned *written);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
