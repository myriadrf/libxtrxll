/*
 * xtrx MMCM reconfiguration source file
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
#include "xtrxll_api.h"
#include "mmcm_rom.h"
#include <errno.h>
#include <unistd.h>
#include "xtrxll_mmcm.h"
#include "xtrxll_base.h"

enum mmcm_regs {
    CLKOUT5_ClkReg1   = 0x06, //CLKOUT5 Register 1
    CLKOUT5_ClkReg2   = 0x07, //CLKOUT5 Register 2
    CLKOUT0_ClkReg1   = 0x08, //CLKOUT0 Register 1
    CLKOUT0_ClkReg2   = 0x09, //CLKOUT0 Register 2
    CLKOUT1_ClkReg1   = 0x0A, //CLKOUT1 Register 1
    CLKOUT1_ClkReg2   = 0x0B, //CLKOUT1 Register 2
    CLKOUT2_ClkReg1   = 0x0C, //CLKOUT2 Register 1 (!PLLE3)
    CLKOUT2_ClkReg2   = 0x0D, //CLKOUT2 Register 2 (!PLLE3)
    CLKOUT3_ClkReg1   = 0x0E, //CLKOUT3 Register 1 (!PLLE3)
    CLKOUT3_ClkReg2   = 0x0F, //CLKOUT3 Register 2 (!PLLE3)
    CLKOUT4_ClkReg1   = 0x10, //CLKOUT4 Register 1 (!PLLE3)
    CLKOUT4_ClkReg2   = 0x11, //CLKOUT4 Register 2 (!PLLE3)
    CLKOUT6_ClkReg1   = 0x12, //CLKOUT6 Register 1 (!PLLE3/2)
    CLKOUT6_ClkReg2   = 0x13, //CLKOUT6 Register 2 (!PLLE3/2)
    CLKFBOUT_ClkReg1  = 0x14, //CLKFBOUT Register 1
    CLKFBOUT_ClkReg2  = 0x15, //CLKFBOUT Register 2
    DIVCLK_DivReg     = 0x16, //DIVCLK Register
    LockReg1          = 0x18, //Lock Register 1
    LockReg2          = 0x19, //Lock Register 2
    LockReg3          = 0x1A, //Lock Register 3
#ifdef ULTRA_SCALE
    PowerReg          = 0x27, //Power Register (UltraScale)
#else
    PowerReg          = 0x28, //Power Register (7 series)
#endif
    FiltReg1          = 0x4E, //Filter Register 1
    FiltReg2          = 0x4F, //Filter Register 2
};

enum mmcm_rom_offs {
    ROM_LOCK_OFF  = 0,
    ROM_FILT_LOW  = 40,
    ROM_FILT_HIGH = 50,
};

enum mmcm_vco_range {
	MMCM_VCO_MIN  =  600000000,
	MMCM_VCO_MAX  = 1200000000,
	MMCM_VCO_MAX2 = 1900000000,
};

// XTRX_MMCM_CLKSEL_OSC   High = CLKIN1, Low = CLKIN2

static int internal_set_txmmcm(struct xtrxll_dev* dev, uint16_t reg, uint16_t value)
{
    struct xtrxll_base_dev* ldev = (struct xtrxll_base_dev*)dev;
    return ldev->ctrlops->set_txmmcm(ldev->self, reg, value);
}

static int internal_get_txmmcm(struct xtrxll_dev* dev, uint16_t* value,
                               uint8_t* locked, uint8_t* rdy)
{
    struct xtrxll_base_dev* ldev = (struct xtrxll_base_dev*)dev;
    return ldev->ctrlops->get_txmmcm(ldev->self, value, locked, rdy);
}

int xtrxll_mmcm_onoff(struct xtrxll_dev* dev, int flags)
{
	int on = (flags & XTRXLL_MMCM_ON) ? 1 : 0;
	uint16_t clksel = (flags & XTRXLL_MMCM_MCLK2) ? 0 : XTRXLL_MMCM_CLKSEL_OSC;
    int res;
    if (on) {
        res = internal_set_txmmcm(dev, XTRXLL_MMCM_RESET | clksel, 0);
        if (res) {
            return res;
        }

        usleep(1000);
        res = internal_set_txmmcm(dev, 0 | clksel, 0);
    } else {
        res = internal_set_txmmcm(dev, XTRXLL_MMCM_PWRDOWN | clksel, 0);
    }
    return res;
}

static int xtrxll_mmcm_trn(struct xtrxll_dev* dev, uint8_t reg, uint16_t in, uint16_t *out)
{
    int res = internal_set_txmmcm(dev, XTRXLL_MMCM_REG_MASK | reg, in);
    unsigned i;
    uint8_t rdy;

    if (res)
        return res;
    for (i = 0; i < 1000; i++) {
        usleep(100);
        res = internal_get_txmmcm(dev, out, NULL, &rdy);
        if (res)
            return res;
        if (rdy)
            return 0;
    }
    return -EFAULT;
}

// phase 0 - 0; 1 - 45; 2 - 90; etc...
static int xtrxll_mmcm_config_clkout(struct xtrxll_dev* dev, uint8_t clk1_reg_num, int div, int phase, int dig_dely)
{
    int res;
    uint16_t clk1_reg_old, clk2_reg_old;
    uint16_t clk1_reg_out, clk2_reg_out;

    res = xtrxll_mmcm_trn(dev, clk1_reg_num, 0, &clk1_reg_old);
    if (res)
        return res;

    clk1_reg_out = ((phase & 7) << 13) |
            (clk1_reg_old & (1 << 12)) |
            (((div / 2) & 0x3f) << 6) |
            (((div + 1) / 2) & 0x3f);

    res = xtrxll_mmcm_trn(dev, clk1_reg_num | XTRXLL_MMCM_WR_MASK, clk1_reg_out, NULL);
    if (res)
        return res;

    res = xtrxll_mmcm_trn(dev, clk1_reg_num + 1, 0, &clk2_reg_old);
    if (res)
        return res;

	clk2_reg_out = (clk2_reg_old & 0xfc00) | /*(1 << 7) |*/ (dig_dely & 0x3f);
			//((div % 2) << 7) |
			//((div <= 1) ? (1 << 6) : 0);

	//res = xtrxll_mmcm_trn(dev, clk2_reg_out | XTRXLL_MMCM_WR_MASK, clk1_reg_out, NULL);
	res = xtrxll_mmcm_trn(dev, (clk1_reg_num + 1) | XTRXLL_MMCM_WR_MASK, clk2_reg_out, NULL);
    if (res)
        return res;

    return 0;
}

static int xtrxll_mmcm_config_div(struct xtrxll_dev* dev, int div)
{
    int res;
    uint16_t tmp, out;

    res = xtrxll_mmcm_trn(dev, DIVCLK_DivReg, 0, &tmp);
    if (res)
        return res;

    out = (tmp & 0xC000) |
            ((div % 2) << 13) |
            ((div <= 1) ? (1 << 12) : 0) |
            (((div / 2) & 0x3f) << 6) |
            (((div + 1) / 2) & 0x3f);

    res = xtrxll_mmcm_trn(dev, DIVCLK_DivReg | XTRXLL_MMCM_WR_MASK, out, NULL);
    if (res)
        return res;

    return 0;
}

// Lock1,2,3
static int xtrxll_mmcm_config_lock(struct xtrxll_dev* dev, int div)
{
    uint16_t tmp, out;
    int res;

    // Lock1
    res = xtrxll_mmcm_trn(dev, LockReg1, 0, &tmp);
    if (res)
        return res;

    out = (tmp & 0xfc00) | ((mmcm_rom[div] >> 20) & 0x3ff);
    res = xtrxll_mmcm_trn(dev, LockReg1 | XTRXLL_MMCM_WR_MASK, out, NULL);
    if (res)
        return res;

    // Lock2
    res = xtrxll_mmcm_trn(dev, LockReg2, 0, &tmp);
    if (res)
        return res;

    out = (tmp & 0x8000) | (((mmcm_rom[div] >> 30) & 0x1f) << 10) | (mmcm_rom[div] & 0x3ff);
    res = xtrxll_mmcm_trn(dev, LockReg2 | XTRXLL_MMCM_WR_MASK, out, NULL);
    if (res)
        return res;

    // Lock3
    res = xtrxll_mmcm_trn(dev, LockReg3, 0, &tmp);
    if (res)
        return res;

    out = (tmp & 0x8000) | (((mmcm_rom[div] >> 35) & 0x1f) << 10) | ((mmcm_rom[div] >> 10) & 0x3ff);
    res = xtrxll_mmcm_trn(dev, LockReg3 | XTRXLL_MMCM_WR_MASK, out, NULL);
    if (res)
        return res;

    return 0;
}

// Filt1,2
static int xtrxll_mmcm_config_filt(struct xtrxll_dev* dev, int div, int h)
{
    uint16_t tmp, out;
    int res;

    unsigned tblval = (h) ? (mmcm_rom[div] >> 50) : ((mmcm_rom[div] >> 40) & 0x3ff);
    // Lock1
    res = xtrxll_mmcm_trn(dev, FiltReg1, 0, &tmp);
    if (res)
        return res;

    out = (((tblval >> 9) & 0x1) << 15) |
            (tmp & 0x6000) |
            (((tblval >> 7) & 0x3) << 11) |
            (tmp & 0x0600) |
            (((tblval >> 6) & 0x1) << 8) |
            (tmp & 0x00ff);

    res = xtrxll_mmcm_trn(dev, FiltReg1 | XTRXLL_MMCM_WR_MASK, out, NULL);
    if (res)
        return res;

    // Lock2
    res = xtrxll_mmcm_trn(dev, FiltReg2, 0, &tmp);
    if (res)
        return res;

    out = (((tblval >> 5) & 0x1) << 15) |
            (tmp & 0x6000) |
            (((tblval >> 3) & 0x3) << 11) |
            (tmp & 0x0600) |
            (((tblval >> 1) & 0x3) << 7) |
            (tmp & 0x0060) |
            ((tblval & 0x1) << 4) |
            (tmp & 0x000f);

    res = xtrxll_mmcm_trn(dev, FiltReg2 | XTRXLL_MMCM_WR_MASK, out, NULL);
    if (res)
        return res;

    return 0;
}

#include <stdio.h>


int xtrxll_mmcm_setfreq(struct xtrxll_dev* dev, int mclk, int rxx2, int txx2, int rx_fwd_delay)
{
    int res, i;
    unsigned div;
    uint8_t lock;

	if (mclk < 1000000)
        return -EINVAL;

	div = (MMCM_VCO_MAX - (mclk/2)) / mclk;
	if (div < 4) {
		div = MMCM_VCO_MAX2 / mclk;
	}

	if (div < 2)
        return -EINVAL;


	if (div % 2)
		div++;

	if ((rxx2 || txx2)&& (div % 4)) {
		div += 2;
	}

	if (div > 64) {
		div = 64;
        if (div * mclk < MMCM_VCO_MIN) {
            printf("div * mclk==%d < MMCM_VCO_MIN==%d\n", div * mclk, MMCM_VCO_MIN);
            //return -EINVAL;
        }
    }


	printf("DIV=%d MMCM_FREQ=%.3f MHZ MCLK=%.3f MHZ RXx2=%d TXx2=%d\n", div, div * mclk / 1.0e6, mclk / 1.0e6, rxx2, txx2);
	//--div;

    // Power
    res = xtrxll_mmcm_trn(dev, PowerReg | XTRXLL_MMCM_WR_MASK, 0xffff, NULL);
    if (res)
        return res;

    // CLKOUT0 - CLKOUT6
	res = xtrxll_mmcm_config_clkout(dev, CLKOUT0_ClkReg1, (txx2) ? div / 2 : div, 0, 0); //0
    if (res)
        return res;

	res = xtrxll_mmcm_config_clkout(dev, CLKOUT1_ClkReg1, (txx2) ? div / 2 : div, 0,  ( (txx2) ? div / 2 : div ) / 4); //2
    if (res)
        return res;

	res = xtrxll_mmcm_config_clkout(dev, CLKOUT2_ClkReg1, (txx2) ? div : div * 2, 0, 0);
    if (res)
        return res;

	res = xtrxll_mmcm_config_clkout(dev, CLKOUT3_ClkReg1, (rxx2) ? div / 2 : div, 0, 0);
    if (res)
        return res;

	res = xtrxll_mmcm_config_clkout(dev, CLKOUT4_ClkReg1, (rxx2) ? div / 2 : div, rx_fwd_delay, 0);
    if (res)
        return res;

	res = xtrxll_mmcm_config_clkout(dev, CLKOUT5_ClkReg1, div, 0, 0);
    if (res)
        return res;

	res = xtrxll_mmcm_config_clkout(dev, CLKOUT6_ClkReg1, div, 0, 0);
    if (res)
        return res;

    // Input divide
	res = xtrxll_mmcm_config_div(dev, 1);
    if (res)
        return res;

    // Feddback divide
	res = xtrxll_mmcm_config_clkout(dev, CLKFBOUT_ClkReg1, div, 0, 0);
    if (res)
        return res;

    // Lock
    res = xtrxll_mmcm_config_lock(dev, div);
    if (res)
        return res;

    // Filter
    res = xtrxll_mmcm_config_filt(dev, div, 1);
    if (res)
        return res;

    // Wait for LOCK
    for (i = 0; i < 100; i++) {
        usleep(1000);
        res = internal_get_txmmcm(dev, NULL, &lock, NULL);
        if (res)
            return res;
        if (lock)
            return 0;
    }

    return -EPROTO;
}
