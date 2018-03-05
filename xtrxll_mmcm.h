/*
 * xtrx MMCM reconfiguration header file
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
#ifndef XTRXLL_MMCM_H
#define XTRXLL_MMCM_H

#include "xtrxll_api.h"
#include <stdbool.h>

enum {
	XTRXLL_MMCM_ON     = 1,
	XTRXLL_MMCM_MCLK2  = 2,
};

enum {
	CLKOUT_COUNT = 7,
	MMCM_DIV_MAX = 128,
	MMCM_DELAY_MAX = 64,
};

typedef struct mmcm_clkout_config {
	uint16_t div;   // divider from VCO
	uint16_t pahse; // total phase offset from VCO in VCO/8 steps
} mmcm_clkout_config_t;

typedef struct mmcm_config {
	unsigned mmcm_port;
	unsigned input_div;
	mmcm_clkout_config_t clkout[CLKOUT_COUNT];
	mmcm_clkout_config_t clkfb;
} mmcm_config_t;

typedef enum lml_clock_mode {
	LML_CLOCK_NORM = 0,
	LML_CLOCK_X2 = 1,
	LML_CLOCK_RX_SELF = 2,
	LML_CLOCK_MODE_MASK = 3,
	LML_CLOCK_FWD_90 = 256,
	LML_CLOCK_INT_X2 = 512,
} lml_clock_mode_t;

// Deprecated
int xtrxll_mmcm_setfreq(struct xtrxll_dev* dev, bool tx, int mclk_freq,
						lml_clock_mode_t mode, int rx_fwd_delay, uint8_t *mdiv, unsigned ndiv);


int xtrxll_mmcm_onoff(struct xtrxll_dev* dev, bool tx, bool on);
int xtrxll_mmcm_set_config(struct xtrxll_dev* dev, const mmcm_config_t* cfg);


int xtrxll_mmcm_fphase_corr(struct xtrxll_dev* dev, bool tx, unsigned gphase, bool fb);

#endif
