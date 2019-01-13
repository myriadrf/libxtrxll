/*
 * xtrx base source file
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
#include "xtrxll_base.h"
#include "xtrxll_log.h"

/* Include verilog file used for FPGA build */
#define localparam static const unsigned
#include "xtrxll_regs.vh"

#include "hw/lp8758.h"

#include <string.h>
#include <memory.h>
#include <stdlib.h>

#define MAKE_I2C_CMD(RD, RDZSZ, WRSZ, DEVNO, DATA)  (\
	(((RD) & 1U) << 31) | \
	(((RDZSZ) & 7U) << 28) | \
	(((WRSZ) & 3U) << 26) | \
	(((DEVNO) & 3U) << 24) | \
	(((DATA) & 0xffffffu) << 0))

enum xtrx_i2c_lut {
	XTRX_I2C_PMIC_FPGA = 0,
	XTRX_I2C_DAC = 1,
	XTRX_I2C_TMP = 2,
	XTRX_I2C_PMIC_LMS = 3,
};

enum fwids {
	FWID_XTRX_R3 = 0,
	FWID_XTRX_R4 = 4,
};

struct internal_base_state {
	uint8_t rx_ant;
	uint8_t tx_ant;

	uint8_t rfic_ctrl;
	uint8_t ext_clk;

	uint8_t gps_state;
	uint8_t gtime_state;
	uint8_t isopps_state;

	uint16_t vio;
	uint16_t v33;
};

_Static_assert( sizeof(struct internal_base_state) <= XTRXLL_INTDATA_STORAGE_ROOM * sizeof(uint32_t), "Not enough space for internal data!");


#define GET_FWID(hwid) ((hwid)>>24)

static int lp8758_get(struct xtrxll_base_dev* dev, uint8_t bus, uint8_t reg,
					  uint8_t* out)
{
	uint32_t tmp;
	int res = dev->selfops->i2c_cmd(dev->self, MAKE_I2C_CMD(1, 0, 1, bus, reg), &tmp);
	if (!res) {
		*out = tmp & 0xff;
	}
	return res;
}

static int lp8758_set(struct xtrxll_base_dev* dev, uint8_t bus, uint8_t reg,
					  uint8_t in)
{
	return dev->selfops->i2c_cmd(dev->self,
								 MAKE_I2C_CMD(0, 0, 2, bus, reg | ((uint32_t)in << 8)), NULL);
}

static int tmp108_get(struct xtrxll_base_dev* dev, unsigned reg, int *outval)
{
	uint32_t tmp = (uint32_t)-1;
	int res = dev->selfops->i2c_cmd(dev->self, MAKE_I2C_CMD(1, 1, 1, XTRX_I2C_TMP, reg),
								&tmp);
	*outval = (int16_t)(htole16(tmp));
	return res;
}

static int ltc2606_set_cur(struct xtrxll_base_dev* dev, unsigned val)
{
	uint32_t cmd = (0x30) | (((val >> 8) & 0xff) << 8) | ((val & 0xff) << 16);
	return dev->selfops->i2c_cmd(dev->self,
								 MAKE_I2C_CMD(0, 0, 3, XTRX_I2C_TMP, cmd), NULL);
}

static int mcp4725_set_cur(struct xtrxll_base_dev* dev, unsigned val)
{
	uint32_t cmd = (((val >> 12) & 0x0f)) | (((val >> 4) & 0xff) << 8);
	return dev->selfops->i2c_cmd(dev->self,
								 MAKE_I2C_CMD(0, 0, 2, XTRX_I2C_DAC, cmd), NULL);
}

static int mcp4725_get_cur(struct xtrxll_base_dev* dev, uint32_t* oval)
{
	return dev->selfops->i2c_cmd(dev->self,
								 MAKE_I2C_CMD(1, 3, 0, XTRX_I2C_DAC, 0), oval);
}


static uint8_t get_voltage(unsigned vin)
{
	if (vin > 3330) {
		return 0xfc;
	} else if (vin > 1400) {
		unsigned x = (vin - 1400) / 20;
		return (uint8_t)(0x9D + x);
	} else if (vin > 730) {
		unsigned x = (vin - 730) / 5;
		return (uint8_t)(0x18 + x);
	} else if (vin > 500) {
		unsigned x = (vin - 500) / 10;
		return (uint8_t)x;
	}

	return 0;
}

enum xtrx_power_modes {
	XTRX_PWR_SAVE_MAX = 0,
	XTRX_PWR_COARSE = 1,
	XTRX_PWR_ECONOMY = 2,
	XTRX_PWR_ECO_PLUS = 3,
	XTRX_PWR_OPTIMAL = 4,
	XTRX_PWR_PERF_GOOD = 5,
	XTRX_PWR_PERF_BEST = 6,
	XTRX_PWR_PERF_MAX = 7,

	XTRX_AUX_SKIP = 0,
	XTRX_AUX_LOW = 1,
	XTRX_AUX_NORM = 2,
	XTRX_AUX_HIGH = 3,
	XTRX_AUX_MAX = 4,
	XTRX_AUX_MAX2 = 5,
	XTRX_AUX_MAX3 = 6,
	XTRX_AUX_MAX4 = 7,

	XTRX_DIG12_SKIP = 0,
	XTRX_DIG12_LOW = 1,
	XTRX_DIG12_NORM = 2,
	XTRX_DIG12_HIGH = 3,
	XTRX_DIG12_MAX = 4,
	XTRX_DIG12_MAX2 = 5,
	XTRX_DIG12_MAX3 = 6,
	XTRX_DIG12_MAX4 = 7,
};

enum {
	XTRX_PWR_V_DROP = 40,
};

enum xtrx_power_out {
	V_LMS_1V8 = 1800,
	V_XTRX_XO = 3200,
	V_LMS_1V4 = 1400,
	V_LMS_1V2 = 1260,
};

enum xtrx_dig_v {
	V_AUX_LOW  = 1700,
	V_AUX_NORM = 1800,
	V_AUX_HIGH = 1900,
	V_AUX_MAX  = 2000,

	V_DIG12_LOW  = 1120,
	V_DIG12_NORM = 1200,
	V_DIG12_HIGH = 1280,
	V_DIG12_MAX  = 1320,
};

enum xtrx_vio {
	V_IO_ECO = 1700,
	V_IO_OPT = 2000,
	V_IO_PERF= 2700,
	V_IO_HI  = 3300,
};


static int _xtrxllr3_io_set(struct xtrxll_base_dev* dev, unsigned vio_mv)
{
	if (vio_mv < 1000 || vio_mv > 3360)
		return -EINVAL;

	int res = lp8758_set(dev, XTRX_I2C_PMIC_FPGA, BUCK1_VOUT,  get_voltage(vio_mv));
	if (res)
		return res;

	struct internal_base_state* intr = (struct internal_base_state*)dev->internal_state;
	intr->vio = vio_mv;
	XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: FPGA V_IO set to %04dmV\n", dev->id, vio_mv);
	return 0;
}

static int _xtrxllr3_gpio_set(struct xtrxll_base_dev* dev, unsigned vgpio_mv)
{
	if (vgpio_mv < 500)
		return -EINVAL;

	if (vgpio_mv > 3360)
		vgpio_mv = 3360;

	int res = lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK1_VOUT,  get_voltage(vgpio_mv));
	if (res)
		return res;

	struct internal_base_state* intr = (struct internal_base_state*)dev->internal_state;
	intr->v33 = vgpio_mv;
	XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: FPGA V_GPIO set to %04dmV\n", dev->id, vgpio_mv);
	return 0;
}


static int _xtrxllr3_d12_set(struct xtrxll_base_dev* dev, unsigned vd12_mv)
{
	if (vd12_mv < 1000 || vd12_mv > 1370)
		return -EINVAL;

	int res = lp8758_set(dev, XTRX_I2C_PMIC_FPGA, BUCK2_VOUT,  get_voltage(vd12_mv));
	if (res)
		return res;

	XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: FPGA V_D12 / LMS DVDD set to %04dmV\n", dev->id, vd12_mv);
	return 0;
}

static int _xtrxllr3_aux_set(struct xtrxll_base_dev* dev, unsigned vaux_mv)
{
	if (vaux_mv < 1600 || vaux_mv > 2100)
		return -EINVAL;

	int res = lp8758_set(dev, XTRX_I2C_PMIC_FPGA, BUCK3_VOUT,  get_voltage(vaux_mv));
	if (res)
		return res;

	XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: FPGA V_AUX set to %04dmV\n", dev->id, vaux_mv);
	return 0;
}

static int _xtrxllr3_pmic_lms_set(struct xtrxll_base_dev* dev, unsigned extra_drop)
{
	int res;
	res = _xtrxllr3_gpio_set(dev, V_XTRX_XO + extra_drop);
	if (res)
		return res;

	res = lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK0_VOUT,  get_voltage(V_LMS_1V8 + extra_drop));
	if (res)
		return res;
/*	res = lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK1_VOUT,  get_voltage(V_XTRX_XO + extra_drop));
	if (res)
		return res;
*/
	res = lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK2_VOUT,  get_voltage(V_LMS_1V4 + extra_drop));
	if (res)
		return res;
	res = lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK3_VOUT,  get_voltage(V_LMS_1V2 + extra_drop));
	if (res)
		return res;

	XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: LMS PMIC DCDC out set to VA18=%04dmV VA14=%04dmV VA12=%04dmV\n",
				dev->id,
			   V_LMS_1V8 + extra_drop,
			   V_LMS_1V4 + extra_drop,
			   V_LMS_1V2 + extra_drop);
	return 0;
}

#define MAXE_MUX(mode, aux, dig12) (((mode) & 0xff) | (((aux) & 0xf) >> 8) | (((dig12) & 0xf) >> 12))

static int _xtrxllr3_lms7pwr_set_mode(struct xtrxll_base_dev* dev, unsigned muxmode)
{
	int res;
	unsigned mode = (muxmode & 0xff);
	unsigned aux_mode = ((muxmode >> 8) & 0x0f);
	unsigned dig12_mode = ((muxmode >> 12) & 0x0f);

	if (mode > XTRX_PWR_PERF_MAX)
		return -EINVAL;
	if (aux_mode > 7)
		return -EINVAL;
	if (dig12_mode > 7)
		return -EINVAL;

	res = _xtrxllr3_pmic_lms_set(dev, XTRX_PWR_V_DROP * mode);
	if (res)
		return res;

	if (aux_mode > 0) {
		unsigned auxv = V_AUX_NORM;
		if (aux_mode == 1) {
			auxv = V_AUX_LOW;
		} else if (aux_mode == 3) {
			auxv = V_AUX_HIGH;
		} else if (aux_mode == 4) {
			auxv = V_AUX_MAX;
		} else if (aux_mode == 5) {
			auxv = V_AUX_MAX + 20;
		} else if (aux_mode == 6) {
			auxv = V_AUX_MAX + 40;
		} else if (aux_mode == 7) {
			auxv = V_AUX_MAX + 60;
		}

		res = _xtrxllr3_aux_set(dev, auxv);
		if (res)
			return res;
	}

	if (dig12_mode > 0) {
		unsigned digv = V_DIG12_NORM;
		if (dig12_mode == 1) {
			digv = V_DIG12_LOW;
		} else if (dig12_mode == 3) {
			digv = V_DIG12_HIGH;
		} else if (dig12_mode == 4) {
			digv = V_DIG12_MAX;
		} else if (dig12_mode == 5) {
			digv = V_DIG12_MAX + 10;
		} else if (dig12_mode == 6) {
			digv = V_DIG12_MAX + 20;
		} else if (dig12_mode == 7) {
			digv = V_DIG12_MAX + 30;
		}

		res = _xtrxllr3_d12_set(dev, digv);
		if (res)
			return res;
	}

	return 0;
}

enum pmic_ctrl {
	PMIC_CH_ENABLE = 0x88,
	PMIC_CH_DISABLE = 0xc8,
};

static int lp8758_en(struct xtrxll_base_dev* dev, int en, int en3v3)
{
	int res;
	uint8_t b_ctrl = (en) ? PMIC_CH_ENABLE : PMIC_CH_DISABLE;

	if (en) {
		// BUS 0 -- LMS PMIC
		res = _xtrxllr3_pmic_lms_set(dev, XTRX_PWR_ECONOMY * XTRX_PWR_V_DROP);
		if (res)
			return res;
	}

	if (en3v3) {
		res = lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK1_CTRL1, (en3v3) ? PMIC_CH_ENABLE : PMIC_CH_DISABLE);
		if (res)
			return res;

		uint8_t v = 0xff, d = 0xff;
		res = lp8758_get(dev->self, XTRX_I2C_PMIC_LMS, DEV_REV, (uint8_t*)&v);
		if (res)
			return res;
		res = lp8758_get(dev->self, XTRX_I2C_PMIC_LMS, OTP_REV, (uint8_t*)&d);
		if (res)
			return res;
		XTRXLLS_LOG("CTRL", XTRXLL_DEBUG, "%s: PMIC_L ver %02x:%02x  en33=%d\n", dev->id, v, d, en3v3);

		for (unsigned i = 0; i < 50; i++) {
			v = 0xff, d = 0xff;
			res = lp8758_get(dev->self, XTRX_I2C_PMIC_FPGA, DEV_REV, (uint8_t*)&v);
			if (res)
				return res;
			res = lp8758_get(dev->self, XTRX_I2C_PMIC_FPGA, OTP_REV, (uint8_t*)&d);
			if (res)
				return res;
			XTRXLLS_LOG("CTRL", XTRXLL_DEBUG, "%s: PMIC_F ver %02x:%02x\n", dev->id, v, d);
			if (v == 0x01 && d == 0xe0)
				break;
		}
	}


	res = lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK0_CTRL1, b_ctrl);
	if (res)
		return res;
	res = lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK2_CTRL1, b_ctrl);
	if (res)
		return res;
	res = lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK3_CTRL1, b_ctrl);
	if (res)
		return res;

	if (en) {
		// BUS 1 -- FPGA PMIC
		res = _xtrxllr3_io_set(dev, 1800);
		if (res)
			return res;
	}

	if (!en3v3) {
		res = lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK1_CTRL1, (en3v3) ? PMIC_CH_ENABLE : PMIC_CH_DISABLE);
		if (res)
			return res;
	}

	return 0;
}

static int _xtrxll_calc_refclk(struct xtrxll_base_dev* dev, unsigned *outclk)
{
	uint32_t t = 0, tmp;
	int res;
	unsigned i, j;
	unsigned cur, prev = 0;
	unsigned pval = 0, cval;
	unsigned delta;
	const unsigned bus_clk = 125000000U;
	*outclk = 0;

	for (i = 0, j = 0; i < 20 && j < 2; i++) {
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_REF_OSC, &tmp);
		if (res)
			return res;

		cur = (tmp >> 28) & 0xf;
		cval = tmp & 0xffffff;

		if (cur == prev) {
			usleep(5000);
			continue;
		}
		if (prev != 0) {
			delta = (pval > cval) ? pval - cval : cval - pval;
		} else {
			delta = (unsigned)-1;
		}

		prev = cur;
		pval = cval;

		if (delta > 4)
			continue;

		t += cval;
		j++;
	}
	if (j < 2)
		return -ENOENT;

	*outclk = (unsigned)(((uint64_t)t * bus_clk) / 65536 / 16 / j);
	return 0;
}

static int _xtrxr3_board_pwr_ctrl(struct xtrxll_base_dev* dev, unsigned param)
{
	int en_lms = (param == PWR_CTRL_ON);
	int en_b33 = (param != PWR_CTRL_PDOWN);

	return lp8758_en(dev, en_lms, en_b33);
}

static int _xtrxr3_board_combctrl(struct xtrxll_base_dev* dev)
{
	struct internal_base_state* intr = (struct internal_base_state*)dev->internal_state;
	uint32_t cmd = 0;

	cmd |= (1<<GP_PORT_XTRX_ENBPVIO_N);
	cmd |= (1<<GP_PORT_XTRX_ENBP3V3_N);

	if (intr->ext_clk == XTRXLL_CLK_EXT)
		cmd |= (1<<GP_PORT_XTRX_EXT_CLK) | (1<<GP_PORT_XTRX_PD_TCXO);
	else if(intr->ext_clk == XTRXLL_CLK_EXT_NOPD)
		cmd |= (1<<GP_PORT_XTRX_EXT_CLK);
	else if(intr->ext_clk == XTRXLL_CLK_INT_PD)
		cmd |= (1<<GP_PORT_XTRX_PD_TCXO);

	cmd |= intr->rfic_ctrl;
	XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: RFIC_GPIO 0x%06x\n",
				dev->id, cmd);
	return dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_LMS_CTRL, cmd);
}

static int xtrvxllv0_get_sensor(struct xtrxll_base_dev* dev, unsigned sensorno, int* outval)
{
	int res;
	uint32_t tmp;
	struct internal_base_state* intr = (struct internal_base_state*)dev->internal_state;

	switch (sensorno) {
	case XTRXLL_CFG_NUM_RFIC:
		*outval = 1;
		return 0;
	case XTRXLL_CFG_HAS_GPS:
		*outval = 1;
		return 0;
	case XTRXLL_CFG_HAS_SIM_READER:
		*outval = 1;
		return 0;

	case XTRXLL_PMIC0_VER:
		*outval = 0;
		return lp8758_get(dev->self, XTRX_I2C_PMIC_FPGA, DEV_REV, (uint8_t*)outval);
	case XTRXLL_PMIC0_ID:
		*outval = 0;
		return lp8758_get(dev->self, XTRX_I2C_PMIC_FPGA, OTP_REV, (uint8_t*)outval);
	case XTRXLL_PMIC1_VER:
		*outval = 0;
		return lp8758_get(dev->self, XTRX_I2C_PMIC_LMS, DEV_REV, (uint8_t*)outval);
	case XTRXLL_PMIC1_ID:
		*outval = 0;
		return lp8758_get(dev->self, XTRX_I2C_PMIC_LMS, OTP_REV, (uint8_t*)outval);

	case XTRXLL_PMIC0_CTRL1:
	case XTRXLL_PMIC1_CTRL1:
	{
#define NUM 4
		uint8_t busno = (sensorno == XTRXLL_PMIC0_CTRL1) ? XTRX_I2C_PMIC_FPGA : XTRX_I2C_PMIC_LMS;
		uint8_t v[NUM];
		uint8_t regs[NUM] = { BUCK0_CTRL1, BUCK1_CTRL1, BUCK2_CTRL1, BUCK3_CTRL1 };
		int i;

		for (i = 0; i < NUM; i++) {
			res = lp8758_get(dev, busno, regs[i], &v[i]);
			if (res) {
				return res;
			}
		}

		*outval = (v[3] << 24) | (v[2] << 16) | (v[1] << 8) | v[0];
		return 0;
	}
	case XTRXLL_TEMP_SENSOR_CUR:
		return tmp108_get(dev->self, 0, outval);
	case XTRXLL_TEMP_SENSOR_MIN:
		return tmp108_get(dev->self, 2, outval);
	case XTRXLL_TEMP_SENSOR_MAX:
		return tmp108_get(dev->self, 3, outval);
	case XTRXLL_OSC_LATCHED:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_ONEPPS, &tmp);
		*outval = (int)(tmp & 0x0fffffff);
		return res;
	case XTRXLL_REFCLK_CNTR:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_REF_OSC, &tmp);
		*outval = (int)tmp;
		return res;
	case XTRXLL_REFCLK_CLK:
		return _xtrxll_calc_refclk(dev->self, (unsigned*)outval);
	case XTRXLL_TEST_CNT_RXIQ_MISS:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_RXIQ_MISS,
								   &tmp);
		*outval = (int)tmp;
		return res;
	case XTRXLL_TEST_CNT_RXIQ_MALGN:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_RXIQ_ODD,
								   &tmp);
		*outval = (int)tmp;
		return res;
	case XTRXLL_HWID:
		*outval = (int)dev->hwid;
		return 0;
	case XTRXLL_DAC_REG:
		res = mcp4725_get_cur(dev->self, &tmp);
		if (res)
			return res;
		*outval = ((tmp) << 8) >> 16;
		return 0;

	case XTRXLL_GTIME_SECFRAC:
		res = dev->selfops->reg_in_n(dev->self, UL_GP_ADDR + GP_PORT_RD_GTIME_SEC,
									 (uint32_t*)outval, 2);
		return res;

	case XTRXLL_GTIME_OFF:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_GTIME_OFF,
								   &tmp);
		*outval = (int)tmp;
		return res;

	case XTRXLL_GPIO_IN:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_GPIO_IN,
								   &tmp);
		*outval = (int)tmp;
		return res;

	case XTRXLL_TX_TIME:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_TXDMA_STATTS,
								   &tmp);
		*outval = (int)tmp;
		return res;

	case XTRXLL_RX_TIME:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_RXDMA_STATTS,
								   &tmp);
		*outval = (int)tmp;
		return res;


	case XTRXLL_XTRX_VIO: *outval = intr->vio; return 0;
	case XTRXLL_XTRX_VGPIO: *outval = intr->v33; return 0;
	case XTRXLL_FE_CTRL: *outval = intr->rfic_ctrl; return 0;
	case XTRXLL_EXT_CLK: *outval = intr->ext_clk; return 0;

	default:
		return -EINVAL;
	}
}

static int xtrvxllv0_lms7_ant(struct xtrxll_base_dev* dev, unsigned rx_ant, unsigned tx_ant)
{
	int res = dev->selfops->reg_out(dev->self,
									UL_GP_ADDR + GP_PORT_WR_RF_SWITCHES,
									((tx_ant & 1) << 2) | (rx_ant & 3) );

	XTRXLLS_LOG("CTRL", XTRXLL_INFO,  "%s: RX_ANT: %d TX_ANT: %d\n",
			   dev->id, (rx_ant & 3), (tx_ant & 1));
	return res;
}

static int xtrvxllv0_drp_set(struct xtrxll_base_dev* dev, unsigned drpno,
			   uint16_t reg, uint16_t value,
			   unsigned drp_gpio, unsigned acc_type)
{
	if (drpno >= MAX_DRPS)
		return -EINVAL;

	uint32_t regout;
	regout = value | (drpno << GP_PORT_DRP_NUM_OFF) |
			(((uint32_t)reg & ((1U << GP_PORT_DRP_ADDR_BITS) - 1)) << GP_PORT_DRP_ADDR_OFF) |
			((drp_gpio & 0xF) << GP_PORT_DRP_GPIO_OFF);

	switch (acc_type) {
	case DRP_SET_REG_WR:
		regout |= (1U << GP_PORT_DRP_REGEN) | (1U << GP_PORT_DRP_REGWR);
		break;
	case DRP_SET_REG_RD:
		regout |= (1U << GP_PORT_DRP_REGEN);
		break;
	case DRP_SET_GPIO:
		break;
	default:
		return -EINVAL;
	}

	return dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_TXMMCM, regout);
}

static int xtrvxllv0_drp_get(struct xtrxll_base_dev* dev, unsigned drpno,
							 uint16_t *reg_value, unsigned* drp_gpio)
{
	if (drpno >= MAX_DRPS)
		return -EINVAL;

	uint32_t regin;
	int res;
	res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_TXMMCM, &regin);
	if (res)
		return res;

	XTRXLLS_LOG("CTRL", XTRXLL_DEBUG, "%s: MMCM -> %04x (%04x.%04x.%04x.%04x)\n",
			   dev->id,
			   regin & 0xffff,
			   (regin >> 16) & 0xf, (regin >> 20) & 0xf,
			   (regin >> 24) & 0xf, (regin >> 28) & 0xf);

	if (reg_value)
		*reg_value = regin & 0xffff;
	if (drp_gpio)
		*drp_gpio = (regin >> (16 + 4 * drpno)) & 0xf;

	return 0;
}

static int xtrvxllv0_issue_timmed_command(struct xtrxll_base_dev* dev,
									   wts32_t time,
									   unsigned route,
									   uint32_t data)
{
	uint32_t stat;
	int res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_TCMDSTAT, &stat);
	if (res)
		return res;

	if (!(stat & (1 << RD_TCMDSTAT_FIFOREADY))) {
		XTRXLLS_LOG("CTRL", XTRXLL_WARNING, "%s: timmend command queue is full\n", dev->id);
		return -EAGAIN;
	}

	XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: Placing TC @%d on %d data: %x stat:%x\n",
							 dev->id, time, route, data, stat);

	res = dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_TCMD_D, data);
	if (res)
		return res;

	res = dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_TCMD_T,
								(time & TS_WTS_INTERNAL_MASK) | (route << TC_TS_BITS));
	if (res)
		return res;

	/* we add faked '| (1 << TC_TS_BITS)' to data to detect incorrect word order in 64 bit writting */
	//internal_xtrxll_reg_out_dual(dev, GP_PORT_WR_TCMD_D, data | (1 << TC_TS_BITS), (time & TS_WTS_INTERNAL_MASK) | (route << TC_TS_BITS));
	return 0;
}

int xtrvxllv0_read_uart(struct xtrxll_base_dev* dev, unsigned uartno, uint8_t* out,
						unsigned maxsize, unsigned *written)
{
	uint32_t rin;
	int res;
	unsigned i;

	for (i =  0; i < maxsize; i++) {
		res = dev->selfops->reg_in(dev->self,
								   UL_GP_ADDR + ((uartno == 0) ? GP_PORT_RD_UART_RX : GP_PORT_RD_SIM_RX),
								   &rin);
		if (res)
			return res;

		if (rin & (1<<UART_FIFORX_EMPTY))
			break;

		out[i] = rin & 0xff;
	}

	*written = i;

	return (i == 0) ? -EAGAIN : 0;
}

enum xtrx_mem_access_cfg {
	MCU_PAGESIZE = 1024,
	MCU_MEM_BURST = 64,
};

static int xtrvxllv0_mem_wr32(struct xtrxll_base_dev* dev,
							  uint32_t xtrx_addr,
							  unsigned mwords, const uint32_t* host_addr)
{
	int res = -EINVAL;
	unsigned i;
	unsigned p = xtrx_addr / (MCU_PAGESIZE / 4);
	//unsigned o = xtrx_addr % (MCU_PAGESIZE / 4);

	res = dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_RF_SWITCHES,
								(1 << MCU_CTRL_VALID) |
								(1 << MCU_CTRL_RESET) |
								(p << MCU_CTRL_PAGESEL));
	if (res)
		return res;

	for (i = 0; i < mwords; i += MCU_MEM_BURST,
							host_addr += MCU_MEM_BURST,
							xtrx_addr += MCU_MEM_BURST) {
		unsigned len = mwords - i;
		if (len > MCU_MEM_BURST)
			len = MCU_MEM_BURST;

		res = dev->selfops->reg_out_n(dev->self,
									  UL_MCU_ADDR + xtrx_addr % (MCU_PAGESIZE / 4),
									  host_addr, len);
		if (res)
			return res;
	}
	return 0;
}

static int xtrvxllv0_mem_rb32(struct xtrxll_base_dev* dev, uint32_t xtrx_addr,
							  unsigned mwords, uint32_t* host_addr)
{
	xtrx_addr &= 0x1ff; // FIXME: TODO (page mapping)

	int res;
	unsigned i;

	for (i = 0; i < mwords; i += MCU_MEM_BURST,
							host_addr += MCU_MEM_BURST,
							xtrx_addr += MCU_MEM_BURST) {
		unsigned len = mwords - i;
		if (len > MCU_MEM_BURST)
			len = MCU_MEM_BURST;

		res = dev->selfops->reg_in_n(dev->self,
									 UL_RD_MEM_ADDR + xtrx_addr % (MCU_PAGESIZE / 4),
									 host_addr, len);
		if (res)
			return res;
	}

	return (int)mwords;
}

static int xtrvxllv0_set_osc_dac(struct xtrxll_base_dev* dev, unsigned val)
{
	if (val > 65535)
		val = 65535;

	if (GET_FWID(dev->hwid) == FWID_XTRX_R4)
		return mcp4725_set_cur(dev, val);

	return ltc2606_set_cur(dev, val);
}

#define TIMECMD_DATA_MASK 0x0fffffff
#define TIMECMD_OFF       28

localparam PPS_CONFIG      = 0;
localparam PPS_CMP_COUNTER = 1;
localparam PPS_OFF         = 2;
localparam PPS_GEN_TMLOW   = 3;
localparam PPS_GEN_TMHI    = 4;


localparam PPS_CFG_OSC_PPS_EXT   = 0;
localparam PPS_CFG_OSC_PPS_EDGE  = 1;
localparam PPS_CFG_TIME_PPS_EXT  = 2;
localparam PPS_CFG_TIME_PPS_EDGE = 3;
localparam PPS_CFG_FWPPS         = 4;
localparam PPS_CFG_ISOG_RESET    = 5;
localparam PPS_CFG_GLOB_RESET    = 6;
localparam PPS_CFG_TIME_RESET    = 7;

static int xtrx_update_timecfg(struct xtrxll_base_dev* dev)
{
	struct internal_base_state* intr = (struct internal_base_state*)dev->internal_state;
	uint32_t config = 0;

	if (intr->gps_state == XTRXLL_PPSDO_EXT_PPS)
		config |= 1 << PPS_CFG_OSC_PPS_EXT;

	if (intr->gtime_state != XTRXLL_GTIME_INT_ISO)
		config |= 1 << PPS_CFG_TIME_PPS_EXT;
	if (intr->gtime_state != XTRXLL_GTIME_EXT_PPS) {
		config |= 1 << PPS_CFG_FWPPS;
	}
	if (intr->gtime_state == XTRXLL_GTIME_DISABLE) {
		config |= 1 << PPS_CFG_GLOB_RESET;
		config |= 1 << PPS_CFG_TIME_RESET;
	}

	if (intr->isopps_state == XTRXLL_GISO_DISABLE) {
		config |= 1 << PPS_CFG_ISOG_RESET;
	}

	XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: TIME CTRL %06x\n", dev->id, config);

	return dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_PPS_CMD,
								 (PPS_CONFIG << TIMECMD_OFF) | (TIMECMD_DATA_MASK & config));

}

static int xtrvxllv0_set_gcmd(struct xtrxll_base_dev* dev,
							  const struct xtrxll_gtime_cmd* cmd)
{
	uint32_t cmd_addr;
	uint32_t cmd_data;

	if (cmd->cmd_idx > 63)
		return -EINVAL;

	switch (cmd->type) {
	case XTRXLL_GCMDT_RFIC_CMD:
		cmd_addr = GP_PORT_WR_SPI_LMS7_0;
		cmd_data = cmd->param;
		break;

	case XTRXLL_GCMDT_TRX_CMD:
		cmd_addr = GP_PORT_WR_RXTXDMA;
		cmd_data = cmd->param;
		break;

	case XTRXLL_GCMDT_RXCMDT_CMD:
		cmd_addr = GP_PORT_WR_TCMD_T;
		cmd_data = cmd->param;
		break;

	case XTRXLL_GCMDT_RXCMDD_CMD:
		cmd_addr = GP_PORT_WR_TCMD_D;
		cmd_data = cmd->param;
		break;

	case XTRXLL_GCMDT_GPIO_SET:
		cmd_addr = GP_PORT_WR_GPIO_OUT;
		cmd_data = cmd->param;
		break;

	case XTRXLL_GCMDT_GPIO_CS:
		cmd_addr = GP_PORT_WR_GPIO_CS;
		cmd_data = cmd->param;
		break;

	default:
		return -EINVAL;
	}

	XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: TIME CMD t:%d %02x: [%04x] <= %08x\n",
				dev->id, cmd->type, cmd->cmd_idx, cmd_addr, cmd_data);

	uint32_t d[2] = {
		((1 << 30) | (cmd->cmd_idx << 24) | cmd_addr),
		cmd_data
	};

	return dev->selfops->reg_out_n(dev->self, UL_GP_ADDR + GP_PORT_WR_GLOBCMDR0,
								   d, 2);
}

localparam TIMEIDX_SEC_W = 14;
localparam TIMEIDX_TICK_W = 26;
localparam TIMEIDX_DATA_W = 6;
localparam TIMEIDX_LEN_W = 2;

static int xtrvxllv0_set_gtime(struct xtrxll_base_dev* dev,
							   const struct xtrxll_gtime_time* cmd)
{
	int res;

	if (cmd->d_idx >= (1U << TIMEIDX_DATA_W))
		return -EINVAL;
	if (cmd->d_cnt > 4 || cmd->d_cnt == 0)
		return -EINVAL;

	if (cmd->frac >= (1U << TIMEIDX_TICK_W))
		return -EINVAL;

	uint64_t tcmd = ((cmd->d_cnt - 1) << 0) |
			(cmd->d_idx << TIMEIDX_LEN_W) |
			(((uint64_t)cmd->frac) << (TIMEIDX_LEN_W + TIMEIDX_DATA_W)) |
			((uint64_t)(cmd->sec & ((1U << TIMEIDX_SEC_W) - 1)) << (TIMEIDX_LEN_W + TIMEIDX_DATA_W + TIMEIDX_TICK_W));


	XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: TIME TIDX (%d) %02x: <= %08d.%08d\n",
				dev->id, cmd->d_cnt, cmd->d_idx, cmd->sec, cmd->frac);

	res = dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_GLOBCMDR0,
								(2U << 30) | (tcmd >> 32));
	if (res)
		return res;

	res = dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_GLOBCMDR1,
								(uint32_t)tcmd);
	if (res)
		return res;

	return 0;
}

static int xtrvxllv0_set_param(struct xtrxll_base_dev* dev, unsigned paramno,
							   uintptr_t param)
{
	struct internal_base_state* intr = (struct internal_base_state*)dev->internal_state;

	switch (paramno) {
	case XTRXLL_PARAM_CLOCK_TYPE:
		return -EINVAL;
	case XTRXLL_PARAM_PWR_MODE:
		return _xtrxllr3_lms7pwr_set_mode(dev, param);
	case XTRXLL_PARAM_PWR_VIO:
		return _xtrxllr3_io_set(dev, param);
	case XTRXLL_PARAM_PWR_VGPIO:
		return _xtrxllr3_gpio_set(dev, param);
	case XTRXLL_PARAM_RX_DLY:
		return dev->selfops->reg_out(dev->self,
									 UL_GP_ADDR + GP_PORT_WR_LMS_CTRL,
									 ((param << 17) | (1 <<16)));
	case XTRXLL_PARAM_REF_DAC:
		return xtrvxllv0_set_osc_dac(dev, param);
	case XTRXLL_PARAM_SWITCH_RX_ANT:
		intr->rx_ant = (uint8_t)param;
		return xtrvxllv0_lms7_ant(dev, intr->rx_ant, intr->tx_ant);
	case XTRXLL_PARAM_SWITCH_TX_ANT:
		intr->tx_ant = (uint8_t)param;
		return xtrvxllv0_lms7_ant(dev, intr->rx_ant, intr->tx_ant);
	case XTRXLL_PARAM_PWR_CTRL:
		return _xtrxr3_board_pwr_ctrl(dev, param);
	case XTRXLL_PARAM_FE_CTRL:
		//if (intr->rfic_ctrl != (uint8_t)param) {
			intr->rfic_ctrl = (uint8_t)param & 0xFF;
			return _xtrxr3_board_combctrl(dev);
		//}
		return 0;
	case XTRXLL_PARAM_EXT_CLK:
		//if (intr->ext_clk != (uint8_t)param) {
			intr->ext_clk = (uint8_t)param & XTRXLL_CLK_MASK;
			return _xtrxr3_board_combctrl(dev);
		//}
		return 0;
	case XTRXLL_PARAM_DSPFE_CMD:
		return dev->selfops->reg_out(dev->self,
									 UL_GP_ADDR + GP_PORT_WR_FE_CMD, param);
	case XTRXLL_PARAM_GPIO_FUNC: {
		if (GET_HWID_COMPAT(dev->hwid) < 1) {
			XTRXLLS_LOG("CTRL", XTRXLL_ERROR, "%s: FPGA Image doesn't support GPIO commands, update it\n", dev->id);
			return -ENOTSUP;
		}

		return dev->selfops->reg_out(dev->self,
									 UL_GP_ADDR + GP_PORT_WR_GPIO_FUNC, param);
	}
	case XTRXLL_PARAM_GPIO_DIR:
		return dev->selfops->reg_out(dev->self,
									 UL_GP_ADDR + GP_PORT_WR_GPIO_DIR, param);
	case XTRXLL_PARAM_GPIO_OUT:
		return dev->selfops->reg_out(dev->self,
									 UL_GP_ADDR + GP_PORT_WR_GPIO_OUT, param);
	case XTRXLL_PARAM_GPIO_CS:
		return dev->selfops->reg_out(dev->self,
									 UL_GP_ADDR + GP_PORT_WR_GPIO_CS, param);

	case XTRXLL_PARAM_PPSDO_CTRL:
		if (param > XTRXLL_PPSDO_EXT_PPS)
			return -EINVAL;
		intr->gps_state = param;
		return xtrx_update_timecfg(dev);

	case XTRXLL_PARAM_GTIME_CTRL:
		if (param > XTRXLL_GTIME_EXT_PPSFW)
			return -EINVAL;
		intr->gtime_state = param;
		return xtrx_update_timecfg(dev);

	case XTRXLL_PARAM_ISOPPS_CTRL:
		if (param > XTRXLL_GISO_PPSFW)
			return -EINVAL;
		intr->isopps_state = param;
		return xtrx_update_timecfg(dev);

	case XTRXLL_PARAM_GTIME_SETCMP:
		return dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_PPS_CMD,
									 (PPS_CMP_COUNTER << TIMECMD_OFF) | (TIMECMD_DATA_MASK & param));

	case XTRXLL_PARAM_GTIME_TRIMOFF:
		return dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_PPS_CMD,
									 (PPS_OFF << TIMECMD_OFF) | (TIMECMD_DATA_MASK & param));

	case XTRXLL_PARAM_ISOPPS_SETTIME:
	{
		int res;
		res = dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_PPS_CMD,
									(PPS_GEN_TMLOW << TIMECMD_OFF) | (TIMECMD_DATA_MASK & param));
		if (res)
			return res;

		return dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_PPS_CMD,
									(PPS_GEN_TMHI << TIMECMD_OFF) | (param >> TIMECMD_OFF));
	}
	case XTRXLL_PARAM_GTIME_RESET:
	{
		// Reset is the best way for the check
		if (GET_HWID_COMPAT(dev->hwid) < 1) {
			XTRXLLS_LOG("CTRL", XTRXLL_ERROR, "%s: FPGA Image doesn't support GTIME commands, update it\n", dev->id);
			return -ENOTSUP;
		}
		return dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_GLOBCMDR0,
									param ? 1 : 0);
	}
	case XTRXLL_PARAM_GTIME_LOAD_CMD:
	{
		const struct xtrxll_gtime_cmd *cmd = (const struct xtrxll_gtime_cmd *)param;
		return xtrvxllv0_set_gcmd(dev, cmd);
	}
	case XTRXLL_PARAM_GTIME_LOAD_TIME:
	{
		const struct xtrxll_gtime_time *tcmd = (const struct xtrxll_gtime_time *)param;
		return xtrvxllv0_set_gtime(dev, tcmd);
	}
	default:
		return -EINVAL;
	}
}

const static struct xtrxll_ctrl_ops s_xtrx_base_ops = {
	.get_sensor = xtrvxllv0_get_sensor,

	.drp_set = xtrvxllv0_drp_set,
	.drp_get = xtrvxllv0_drp_get,

	.issue_timmed_command = xtrvxllv0_issue_timmed_command,

	.read_uart = xtrvxllv0_read_uart,

	.mem_rb32 = xtrvxllv0_mem_rb32,
	.mem_wr32 = xtrvxllv0_mem_wr32,

	.set_param = xtrvxllv0_set_param,
};


int xtrxll_base_dev_init(struct xtrxll_base_dev* dev,
						  const struct xtrxll_ops* ops,
						  const char* id)
{
	dev->self = dev;
	dev->selfops = ops;
	dev->id = id;
	dev->ctrlops = &s_xtrx_base_ops;
	memset(&dev->internal_state, 0, sizeof(dev->internal_state));

	int res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_HWCFG,
											  &dev->hwid);
	if (res)
		return res;

	switch (GET_FWID(dev->hwid)) {
	case FWID_XTRX_R3:
		XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: XTRX Rev3 (%08x)\n", dev->id, dev->hwid);
		return 0;
	case FWID_XTRX_R4:
		XTRXLLS_LOG("CTRL", XTRXLL_INFO, "%s: XTRX Rev4 (%08x)\n", dev->id, dev->hwid);
		return 0;
	}

	XTRXLLS_LOG("CTRL", XTRXLL_ERROR, "%s: Unrecognized HWID %08x!\n", dev->id, dev->hwid);
	return -ENOTSUP;
}
