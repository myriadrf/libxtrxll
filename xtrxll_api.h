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

// LMS7 control
enum xtrxll_lms7_pwr {
	XTRXLL_LMS7_RESET_PIN = 1<<1,
	XTRXLL_LMS7_GPWR_PIN  = 1<<2,
	XTRXLL_LMS7_RXEN_PIN  = 1<<3,
	XTRXLL_LMS7_TXEN_PIN  = 1<<4,

	XTRXLL_LMS7_RX_GEN    = 1<<6,
	XTRXLL_LMS7_RX_TERM_D = 1<<7,

	XTRXLL_EXT_CLK        = 1<<8,

	XTRXLL_DCDC_ON        = 1<<16,
};

XTRXLL_API int xtrxll_lms7_pwr_ctrl(struct xtrxll_dev* dev, uint32_t lmsno,
									unsigned ctrl_mask);

enum xtrxll_sensors {
	XTRXLL_TEMP_SENSOR_CUR,
	XTRXLL_TEMP_SENSOR_MAX,
	XTRXLL_TEMP_SENSOR_MIN,
	XTRXLL_OSC_LATCHED,
	XTRXLL_ONEPPS_CAPTURED,
	XTRXLL_REFCLK_CNTR,
	XTRXLL_REFCLK_CLK,

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

	XTRXLL_CFG_NUM_RFIC,
	XTRXLL_CFG_HAS_GPS,
	XTRXLL_CFG_HAS_SIM_READER,
};

XTRXLL_API int xtrxll_get_sensor(struct xtrxll_dev* dev, unsigned sensorno,
								 int* outval);

typedef enum xtrxll_params {
	XTRXLL_PARAM_CLOCK_TYPE,
	XTRXLL_PARAM_PWR_MODE,
	XTRXLL_PARAM_PWR_VIO,
	XTRXLL_PARAM_RX_DLY,
	XTRXLL_PARAM_REF_DAC,
	XTRXLL_PARAM_SWITCH_RX_ANT,
	XTRXLL_PARAM_SWITCH_TX_ANT,
	XTRXLL_PARAM_PWR_CTRL,
	XTRXLL_PARAM_EXT_CLK,

} xtrxll_params_t;

XTRXLL_API int xtrxll_set_param(struct xtrxll_dev* dev, unsigned paramno,
								unsigned value);

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

	XTRXLL_FE_MODE_4X_ACC_OVER  = 4,
	XTRXLL_FE_MODE_16X_ACC_OVER = 8,
	XTRXLL_FE_MODE_16X_NO_ACC   = XTRXLL_FE_MODE_4X_ACC_OVER | XTRXLL_FE_MODE_16X_ACC_OVER,
	XTRXLL_FE_MODE_OVER_MASK    = XTRXLL_FE_MODE_16X_NO_ACC,

	XTRXLL_FE_MODE_INTER_OFF  = 8,
	XTRXLL_FE_MODE_INTER_MASK = 7,

} xtrxll_mode_t;

XTRXLL_API int xtrxll_dma_start(struct xtrxll_dev* dev, int chan,
								xtrxll_fe_t rxfe, xtrxll_mode_t rxmode,
								wts_long_t rx_start_sample,
								xtrxll_fe_t txfe, xtrxll_mode_t txmode);

static inline int xtrxll_dma_rx_start(struct xtrxll_dev* dev, int chan,
									  xtrxll_fe_t fe)
{
	return xtrxll_dma_start(dev, chan, fe, XTRXLL_FE_MODE_MIMO, 0,
							XTRXLL_FE_DONTTOUCH, XTRXLL_FE_MODE_MIMO);
}


static inline int xtrxll_dma_tx_start(struct xtrxll_dev* dev, int chan,
									  xtrxll_fe_t fe, xtrxll_mode_t mode)
{
	return xtrxll_dma_start(dev, chan, XTRXLL_FE_DONTTOUCH,
							XTRXLL_FE_MODE_MIMO, 0, fe, mode);
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
