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
#include <stdbool.h>
#include "xtrxll_mmcm.h"
#include "xtrxll_base.h"
#include "xtrxll_log.h"

/* Include verilog file used for FPGA build */
#define localparam static const unsigned
#include "xtrxll_regs.vh"

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
	MMCM_VCO_MAX  = 1440000000,
	MMCM_VCO_MAX2 = 1900000000,
};

// XTRX_MMCM_CLKSEL_OSC   High = CLKIN1, Low = CLKIN2

static int internal_set_txmmcm(struct xtrxll_dev* dev,
							   unsigned drpno,
							   uint16_t reg,
							   uint16_t value,
							   unsigned mmcmgpios,
							   unsigned drpflags)
{
	struct xtrxll_base_dev* ldev = (struct xtrxll_base_dev*)dev;
	return ldev->ctrlops->drp_set(ldev->self, drpno, reg, value, mmcmgpios, drpflags);
}

static int internal_get_txmmcm(struct xtrxll_dev* dev, unsigned drpno,
							   uint16_t* value, uint8_t* locked,
							   uint8_t* instopped, uint8_t* fbstopped)
{
	struct xtrxll_base_dev* ldev = (struct xtrxll_base_dev*)dev;
	unsigned drpgpio;
	int res = ldev->ctrlops->drp_get(ldev->self, drpno, value, &drpgpio);
	if (res) {
		if (locked)
			*locked = 0;
		return res;
	}

	if (locked)
		*locked = (drpgpio & (1U << GP_PORT_IN_MMCM_LOCKED));
	if (instopped)
		*instopped = (drpgpio & (1U << GP_PORT_IN_MMCM_STOPPED));
	if (fbstopped)
		*fbstopped = (drpgpio & (1U << GP_PORT_IN_MMCM_STOPPEDF));
	return 0;
}

int xtrxll_mmcm_onoff(struct xtrxll_dev* dev, bool tx, bool on)
{
#if 0
	int on = (flags & XTRXLL_MMCM_ON) ? 1 : 0;
	uint16_t clksel = (flags & XTRXLL_MMCM_MCLK2) ? 0 : XTRXLL_MMCM_CLKSEL_OSC;
	return internal_set_txmmcm(dev, (on ? XTRXLL_MMCM_RESET : XTRXLL_MMCM_PWRDOWN | XTRXLL_MMCM_RESET) | clksel, 0);
#endif

	unsigned mmcm_port = (tx) ? DRP_PORT_MMCM_TX : DRP_PORT_MMCM_RX;
	unsigned mmcm_gpio = (1U << GP_PORT_OUT_MMCM_RESET) |
			(1U << GP_PORT_OUT_MMCM_CLKSEL1) |
			(on ? 0 : (1U << GP_PORT_OUT_MMCM_PWRDOWN));
	return internal_set_txmmcm(dev, mmcm_port, 0, 0, mmcm_gpio, DRP_SET_GPIO);
}

static int xtrxll_mmcm_trn(struct xtrxll_dev* dev, unsigned drpport, uint8_t reg,
						   uint16_t in, uint16_t *out)
{
	unsigned i;
	//uint8_t rdy;

	int res;
	//res = internal_set_txmmcm(dev, XTRXLL_MMCM_REG_MASK | reg, in);
	res = internal_set_txmmcm(dev, drpport, reg, in, 0,
							  (out) ? DRP_SET_REG_RD : DRP_SET_REG_WR);
	if (res)
		return res;

	for (i = 0; i < 10000; i++) {
		//usleep(100);
		//res = internal_get_txmmcm(dev, out, NULL, &rdy);
		res = internal_get_txmmcm(dev, drpport, out, NULL, NULL, NULL);
		if (res)
			return res;
		//if (rdy) {
			//res = internal_get_txmmcm(dev, out, NULL, &rdy);
			if (out) {
				XTRXLL_LOG(XTRXLL_DEBUG, "MMCM RD reg %02x => %04x\n", reg, *out);
			} else {
				XTRXLL_LOG(XTRXLL_DEBUG, "MMCM WR reg %02x <= %04x\n", reg, in);
			}
			return res;
		//}
	}
	XTRXLL_LOG(XTRXLL_WARNING, "MMCM: reg %02x timed out!\n", reg);
	return -EFAULT;
}

// phase 0 - 0; 1 - 45; 2 - 90; etc...
static int xtrxll_mmcm_config_clkout(struct xtrxll_dev* dev, unsigned drpport,
									 uint8_t clk1_reg_num,
									 int div, int phase, int dig_dely)
{
	int res;
	uint16_t clk1_reg_old, clk2_reg_old;
	uint16_t clk1_reg_out, clk2_reg_out;

	res = xtrxll_mmcm_trn(dev, drpport, clk1_reg_num, 0, &clk1_reg_old);
	if (res)
		return res;

	clk1_reg_out = ((phase & 7) << 13) |
			(clk1_reg_old & (1 << 12)) |
			(((div / 2) & 0x3f) << 6) |
			(((div + 1) / 2) & 0x3f);

	res = xtrxll_mmcm_trn(dev, drpport, clk1_reg_num, clk1_reg_out, NULL);
	if (res)
		return res;

	res = xtrxll_mmcm_trn(dev, drpport, clk1_reg_num + 1, 0, &clk2_reg_old);
	if (res)
		return res;

	clk2_reg_out = (clk2_reg_old & 0xff00) | ((div % 2) << 7) | (dig_dely & 0x3f);
	//((div <= 1) ? (1 << 6) : 0);

	res = xtrxll_mmcm_trn(dev, drpport, clk1_reg_num + 1, clk2_reg_out, NULL);
	if (res)
		return res;


	XTRXLL_LOG(XTRXLL_DEBUG, "CLKREG %02x OLD: PHASE=%d HIGH=%d LOW=%d | MX=%d EDGE=%d NO_CNT=%d DELAY=%d\n",
			   clk1_reg_num,
			   (clk1_reg_old >> 13) & 0x7, (clk1_reg_old >> 6) & 0x3f, clk1_reg_old & 0x3f,
			   (clk2_reg_old >> 8) & 0x3, (clk2_reg_old >> 7) & 1, (clk2_reg_old >> 6) & 1,
			   (clk2_reg_old & 0x3f));
	return 0;
}

static int xtrxll_mmcm_config_div(struct xtrxll_dev* dev, unsigned drpport, int div)
{
	int res;
	uint16_t tmp, out;

	res = xtrxll_mmcm_trn(dev, drpport, DIVCLK_DivReg, 0, &tmp);
	if (res)
		return res;

	out = (tmp & 0xC000) |
			((div % 2) << 13) |
			((div <= 1) ? (1 << 12) : 0) |
			(((div / 2) & 0x3f) << 6) |
			(((div + 1) / 2) & 0x3f);

	res = xtrxll_mmcm_trn(dev, drpport, DIVCLK_DivReg, out, NULL);
	if (res)
		return res;
	return 0;
}

// Lock1,2,3
static int xtrxll_mmcm_config_lock(struct xtrxll_dev* dev, unsigned drpport, int div)
{
    uint16_t tmp, out;
    int res;

    // Lock1
	res = xtrxll_mmcm_trn(dev, drpport, LockReg1, 0, &tmp);
    if (res)
        return res;

    out = (tmp & 0xfc00) | ((mmcm_rom[div] >> 20) & 0x3ff);
	res = xtrxll_mmcm_trn(dev, drpport, LockReg1, out, NULL);
    if (res)
        return res;

    // Lock2
	res = xtrxll_mmcm_trn(dev, drpport, LockReg2, 0, &tmp);
    if (res)
        return res;

    out = (tmp & 0x8000) | (((mmcm_rom[div] >> 30) & 0x1f) << 10) | (mmcm_rom[div] & 0x3ff);
	res = xtrxll_mmcm_trn(dev, drpport, LockReg2, out, NULL);
    if (res)
        return res;

    // Lock3
	res = xtrxll_mmcm_trn(dev, drpport, LockReg3, 0, &tmp);
    if (res)
        return res;

    out = (tmp & 0x8000) | (((mmcm_rom[div] >> 35) & 0x1f) << 10) | ((mmcm_rom[div] >> 10) & 0x3ff);
	res = xtrxll_mmcm_trn(dev, drpport, LockReg3, out, NULL);
    if (res)
        return res;

    return 0;
}

// Filt1,2
static int xtrxll_mmcm_config_filt(struct xtrxll_dev* dev, unsigned drpport,
								   int div, int h)
{
    uint16_t tmp, out;
    int res;

    unsigned tblval = (h) ? (mmcm_rom[div] >> 50) : ((mmcm_rom[div] >> 40) & 0x3ff);
    // Lock1
	res = xtrxll_mmcm_trn(dev, drpport, FiltReg1, 0, &tmp);
    if (res)
        return res;

    out = (((tblval >> 9) & 0x1) << 15) |
            (tmp & 0x6000) |
            (((tblval >> 7) & 0x3) << 11) |
            (tmp & 0x0600) |
            (((tblval >> 6) & 0x1) << 8) |
            (tmp & 0x00ff);

	res = xtrxll_mmcm_trn(dev, drpport, FiltReg1, out, NULL);
    if (res)
        return res;

    // Lock2
	res = xtrxll_mmcm_trn(dev, drpport, FiltReg2, 0, &tmp);
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

	res = xtrxll_mmcm_trn(dev, drpport, FiltReg2, out, NULL);
    if (res)
        return res;

    return 0;
}

int xtrxll_mmcm_set_config(struct xtrxll_dev* dev, const mmcm_config_t* cfg)
{
	int res;
	if (cfg->mmcm_port >= 4)
		return -EINVAL;
	if (cfg->input_div == 0)
		return -EINVAL;
	if (cfg->clkfb.div == 0 || cfg->clkfb.div > MMCM_DIV_MAX ||
			cfg->clkfb.pahse >= MMCM_DELAY_MAX * 8) {
		XTRXLL_LOG(XTRXLL_ERROR, "MMCM: ClkFb incorrect settings\n");
		return -EINVAL;
	}

	// Power
	res = xtrxll_mmcm_trn(dev, cfg->mmcm_port, PowerReg, 0xffff, NULL);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "MMCM: unable to turn it on\n");
		return res;
	}

	// CLKOUT0 - CLKOUT6
	const uint8_t clkreg_map[CLKOUT_COUNT] = {
		CLKOUT0_ClkReg1,
		CLKOUT1_ClkReg1,
		CLKOUT2_ClkReg1,
		CLKOUT3_ClkReg1,
		CLKOUT4_ClkReg1,
		CLKOUT5_ClkReg1,
		CLKOUT6_ClkReg1,
	};
	for (unsigned i = 0; i < CLKOUT_COUNT; i++) {
		if (cfg->clkout[i].div == 0 || cfg->clkout[i].div > MMCM_DIV_MAX ||
				cfg->clkout[i].pahse >= MMCM_DELAY_MAX * 8) {
			XTRXLL_LOG(XTRXLL_ERROR, "MMCM: ClkOut%u incorrect settings\n", i);
			return -EINVAL;
		}

		res = xtrxll_mmcm_config_clkout(dev, cfg->mmcm_port, clkreg_map[i],
										cfg->clkout[i].div,
										cfg->clkout[i].pahse % 8,
										cfg->clkout[i].pahse / 8);
		if (res)
			return res;
	}

	// Input divide
	res = xtrxll_mmcm_config_div(dev, cfg->mmcm_port, cfg->input_div);
	if (res)
		return res;

	// Feddback divide
	res = xtrxll_mmcm_config_clkout(dev, cfg->mmcm_port, CLKFBOUT_ClkReg1,
									cfg->clkfb.div,
									cfg->clkfb.pahse % 8,
									cfg->clkfb.pahse / 8);
	if (res)
		return res;

	// Lock
	res = xtrxll_mmcm_config_lock(dev, cfg->mmcm_port, cfg->clkfb.div);
	if (res)
		return res;

	// Filter
	res = xtrxll_mmcm_config_filt(dev, cfg->mmcm_port, cfg->clkfb.div, 1);
	if (res)
		return res;

	return 0;
}

int xtrxll_mmcm_fphase_corr(struct xtrxll_dev* dev, bool tx, unsigned gphase, bool fb)
{
	unsigned drpport = (tx) ? DRP_PORT_MMCM_TX : DRP_PORT_MMCM_RX;
	int res;
	unsigned clk1_reg_num = (fb) ? CLKFBOUT_ClkReg1 : CLKOUT0_ClkReg1;
	unsigned phase = gphase % 8;
	unsigned dig_dely = (gphase / 8);

	uint16_t clk1_reg_old, clk2_reg_old;
	uint16_t clk1_reg_out, clk2_reg_out;

	res = xtrxll_mmcm_trn(dev, drpport, clk1_reg_num, 0, &clk1_reg_old);
	if (res)
		return res;

	clk1_reg_out = ((phase & 7) << 13) | (clk1_reg_old & 0x1fff);

	res = xtrxll_mmcm_trn(dev, drpport, clk1_reg_num, clk1_reg_out, NULL);
	if (res)
		return res;

	res = xtrxll_mmcm_trn(dev, drpport, clk1_reg_num + 1, 0, &clk2_reg_old);
	if (res)
		return res;

	clk2_reg_out = (clk2_reg_old & 0xffc0) | (dig_dely & 0x3f);

	res = xtrxll_mmcm_trn(dev, drpport, clk1_reg_num + 1, clk2_reg_out, NULL);
	if (res)
		return res;


	XTRXLL_LOG(XTRXLL_WARNING, "PHASE_CORR CLKREG %02x OLD: PHASE=%d HIGH=%d LOW=%d | MX=%d EDGE=%d NO_CNT=%d DELAY=%d\n",
			   clk1_reg_num,
			   (clk1_reg_old >> 13) & 0x7, (clk1_reg_old >> 6) & 0x3f, clk1_reg_old & 0x3f,
			   (clk2_reg_old >> 8) & 0x3, (clk2_reg_old >> 7) & 1, (clk2_reg_old >> 6) & 1,
			   (clk2_reg_old & 0x3f));

	return res;
}

int xtrxll_mmcm_setfreq(struct xtrxll_dev* dev, bool tx, int mclk,
						lml_clock_mode_t rmode, int rx_fwd_delay,
						uint8_t *mdiv, unsigned ndiv)
{
	int res, i;
	unsigned div;
	uint8_t lock;
	const unsigned mode = rmode & LML_CLOCK_MODE_MASK;
	const bool xn = (mode == LML_CLOCK_X2);
	const bool flag_x2 = ((rmode & LML_CLOCK_INT_X2) == LML_CLOCK_INT_X2);

	if (mclk < 1000000)
		return -EINVAL;

	div = (MMCM_VCO_MAX2 - (mclk/2)) / mclk;
	if (div < 4) {
		div = MMCM_VCO_MAX2 / mclk;
	}

	if (div < 2)
		return -EINVAL;

	if ((xn) && (div % 2)) {
		div++;
	}

	unsigned mmcm_max_div = 1;
	if (xn || flag_x2) {
		if (xn) {
			if (ndiv < 2)
				ndiv = 2;
		} else {
			ndiv = 1;
		}
		mmcm_max_div = (flag_x2 ? 2 : 1) * (xn ? ndiv : 1);

		if (div % mmcm_max_div) {
			div += (mmcm_max_div - (div % mmcm_max_div));
		}
	} else {
		ndiv = 0;
	}

	/* 64 and above are unstable */
	if (div > 62) {
		div = 62;
		if (mmcm_max_div > 1) {
			div -= (div % mmcm_max_div);
		}

		if (div * mclk < MMCM_VCO_MIN) {
			XTRXLL_LOG(XTRXLL_WARNING, "MMCM: div * mclk==%d < MMCM_VCO_MIN==%d (mmcm_max_div=%d)\n",
					   div * mclk, MMCM_VCO_MIN, mmcm_max_div);
		}
	}

	unsigned mmcm_port = (tx) ? DRP_PORT_MMCM_TX : DRP_PORT_MMCM_RX;
	res = internal_set_txmmcm(dev, mmcm_port, 0, 0,
							  (1U << GP_PORT_OUT_MMCM_RESET), DRP_SET_GPIO);
	if (res)
		return res;

	usleep(1000);

	XTRXLL_LOG(XTRXLL_WARNING, "MMCM: DIV=%d/%d MMCM_FREQ=%.3f MHZ MCLK=%.3f MHZ TX=%d X2=%d div=%d/%d\n",
			   div, ndiv, div * mclk / 1.0e6, mclk / 1.0e6, tx, xn, div, mmcm_max_div);
	//--div;
	mmcm_config_t config;
	config.mmcm_port = mmcm_port;
	config.input_div = 1;
	config.clkout[0].div = (xn) ? div / ndiv : div;
	config.clkout[0].pahse = (rmode & LML_CLOCK_FWD_90) ? (8 * ( (xn) ? div / ndiv : div ) / 4) : ((rx_fwd_delay & 0x3ff)); //90DEG for TX
	config.clkout[1].div = ((xn) ? div / ndiv : div) / (flag_x2 ? 2 : 1);
	config.clkout[1].pahse = 0;
	config.clkout[2].div = ((xn) ? div / ndiv : div) / (flag_x2 ? 2 : 1);
	config.clkout[2].pahse = 0; //((xn) ? div / ndiv : div) - 1;
	config.clkout[3].div = (xn) ? div / ndiv : div;
	config.clkout[3].pahse = 0;
	config.clkout[4].div = (xn) ? div / ndiv : div;
	config.clkout[4].pahse = 0;
	config.clkout[5].div = (xn) ? div / ndiv : div;
	config.clkout[5].pahse = 0;
	config.clkout[6].div = 2;   // CASCADE for CLKOUT4
	config.clkout[6].pahse = 0; // CASCADE for CLKOUT4
	config.clkfb.div = div;
	config.clkfb.pahse = (mode == LML_CLOCK_RX_SELF) ? (rx_fwd_delay & 0x3ff) : ((rx_fwd_delay >> 10) & 0x3ff);

	res = xtrxll_mmcm_set_config(dev, &config);
	if (res) {
		XTRXLL_LOG(XTRXLL_ERROR, "MMCM: xtrxll_mmcm_set_config failed: res %d\n", res);
		return res;
	}

	res = internal_set_txmmcm(dev, mmcm_port, 0, 0, 0, DRP_SET_GPIO);
	if (res)
		return res;

	if (mdiv)
		*mdiv = div;

	// Wait for LOCK
	uint8_t in_stp, fb_stp;
	for (i = 0; i < 500; i++) {
		usleep(1000);
		res = internal_get_txmmcm(dev, mmcm_port, NULL, &lock, &in_stp, &fb_stp);
		if (res)
			return res;

		if (in_stp || fb_stp) {
			XTRXLL_LOG(XTRXLL_WARNING, "MMCM failed: FB_loss:%d IN_loss:%d\n", fb_stp, in_stp);

			//Need to reset MMCM in case of clock loss
			res = internal_set_txmmcm(dev, mmcm_port, 0, 0,
									  (1U << GP_PORT_OUT_MMCM_RESET), DRP_SET_GPIO);
			if (res)
				return res;

			usleep(100);

			res = internal_set_txmmcm(dev, mmcm_port, 0, 0, 0, DRP_SET_GPIO);
			if (res)
				return res;

			continue;
		}

		if (lock) {
			return 0;
		}
	}

	XTRXLL_LOG(XTRXLL_ERROR, "MMCM: timed out waiting for lock: FB=%d IN=%d;"
			   " DIV=%d MMCM_FREQ=%.3f MHZ MCLK=%.3f MHZ TX=%d X2=%d\n",
			   fb_stp, in_stp,
			   div, div * mclk / 1.0e6, mclk / 1.0e6, tx, xn);
	return -EFAULT;
}
