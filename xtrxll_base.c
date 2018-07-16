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
	return dev->selfops->i2c_cmd(dev->self, MAKE_I2C_CMD(1, 3, 0, XTRX_I2C_DAC, 0), oval);
}


static uint8_t get_voltage(int vin)
{
	if (vin > 3330) {
		return 0xfc;
	} else if (vin > 1400) {
		int x = (vin - 1400) / 20;
		return 0x9D + x;
	} else if (vin > 730) {
		int x = (vin - 730) / 5;
		return 0x18 + x;
	} else if (vin > 500) {
		int x = (vin - 500) / 10;
		return x;
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
};

enum xtrx_power_out {
	V_LMS_1V8 = 1800,
	V_XTRX_XO = 3000 + 20,
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

static int xtrxllr3_lms7pwr_set_mode(struct xtrxll_base_dev* dev, unsigned muxmode)
{
	unsigned mode = (muxmode & 0xff);
	unsigned aux_mode = ((muxmode >> 8) & 0x0f);
	unsigned dig12_mode = ((muxmode >> 12) & 0x0f);

	if (mode > XTRX_PWR_PERF_MAX)
		return -EINVAL;
	if (aux_mode > 7)
		return -EINVAL;
	if (dig12_mode > 7)
		return -EINVAL;

	unsigned drop_voltage = 40 * mode;
	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK0_VOUT,  get_voltage(V_LMS_1V8 + drop_voltage));
	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK1_VOUT,  get_voltage(V_XTRX_XO + drop_voltage));
	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK2_VOUT,  get_voltage(V_LMS_1V4 + drop_voltage));
	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK3_VOUT,  get_voltage(V_LMS_1V2 + drop_voltage));

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

		lp8758_set(dev, XTRX_I2C_PMIC_FPGA, BUCK3_VOUT,  get_voltage(auxv));
		XTRXLL_LOG(XTRXLL_WARNING, "V_AUX set to %d\n", auxv);
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

		lp8758_set(dev, XTRX_I2C_PMIC_FPGA, BUCK2_VOUT,  get_voltage(digv));
		XTRXLL_LOG(XTRXLL_WARNING, "V_D12 set to %d\n", digv);
	}

	return 0;
}

static int xtrxllr3_io_set(struct xtrxll_base_dev* dev, unsigned vio_mv)
{
	if (vio_mv < 1000 || vio_mv > 3300)
		return -EINVAL;

	lp8758_set(dev, XTRX_I2C_PMIC_FPGA, BUCK1_VOUT,  get_voltage(vio_mv));
	XTRXLL_LOG(XTRXLL_WARNING, "V_IO set to %d\n", vio_mv);
	return 0;
}


static void lp8758_en(struct xtrxll_base_dev* dev, int en, int en3v3)
{
	// Set voltages to channels
	// 0 - 2.05          -> 1.8
	// 1 - 3.3 (+bypass) -> 3.0
	// 2 - 1.65          -> 1.4
	// 3 - 1.5           -> 1.25

	uint8_t b_ctrl = (en) ? 0x88 : 0xc8;

	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK0_VOUT, 0x9D + 32 - 2);
	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK1_VOUT, 0x9D + 95);
	//lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK1_VOUT, 0x9D + 30); Can be lowered up to 2v0
	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK2_VOUT, 0x9D + 12 - 2);
	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK3_VOUT, 0x9D + 5 - 2);

	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK0_CTRL1, b_ctrl);
	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK1_CTRL1, (en3v3) ? 0x88 : 0xc8);
	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK2_CTRL1, b_ctrl);
	lp8758_set(dev, XTRX_I2C_PMIC_LMS, BUCK3_CTRL1, b_ctrl);

	// BUS 1
	// lp8758_set(dev, XTRX_I2C_PMIC_FPGA, BUCK1_VOUT,  get_voltage(3300));
	lp8758_set(dev, XTRX_I2C_PMIC_FPGA, BUCK1_CTRL1, 0x88);

	if (en) {
		// Wait for power ramp up
		usleep(1000);
	}
}



static int xtrvxllv0_get_osc_freq(struct xtrxll_base_dev* dev, uint32_t *regval)
{
	return dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_ONEPPS, regval);
}

static int xtrvxllv0_lms7_pwr_ctrl(struct xtrxll_base_dev* dev, uint32_t lmsno,
								   unsigned ctrl_mask)
{
	int res;
	if (!(lmsno & XTRXLL_LMS7_0)) {
		return -EINVAL;
	}

	uint32_t powermask = 0;

	powermask |= (1<<GP_PORT_XTRX_ENBPVIO_N);
	powermask |= (1<<GP_PORT_XTRX_ENBP3V3_N);


	if (ctrl_mask & XTRXLL_LMS7_RESET_PIN) powermask |= (1<<GP_PORT_LMS_CTRL_RESET) | (1<<GP_PORT_LMS_CTRL_DIGRESET);
	if (ctrl_mask & XTRXLL_LMS7_GPWR_PIN)  powermask |= (1<<GP_PORT_LMS_CTRL_GPWR);
	if (ctrl_mask & XTRXLL_LMS7_RXEN_PIN)  powermask |= (1<<GP_PORT_LMS_CTRL_RXEN);
	if (ctrl_mask & XTRXLL_LMS7_TXEN_PIN)  powermask |= (1<<GP_PORT_LMS_CTRL_TXEN);

	if (ctrl_mask & XTRXLL_LMS7_RX_GEN)    powermask |= (1<<GP_PORT_LMS_FCLK_RX_GEN);
	if (ctrl_mask & XTRXLL_LMS7_RX_TERM_D) powermask |= (1<<GP_PORT_LMS_RX_TERM_DIS);

	if (ctrl_mask & XTRXLL_EXT_CLK)        powermask |= (1<<GP_PORT_XTRX_EXT_CLK) | (1<<GP_PORT_XTRX_PD_TCXO);

	if ((ctrl_mask & XTRXLL_LMS7_GPWR_PIN) || (ctrl_mask & XTRXLL_DCDC_ON)) {
		lp8758_en(dev, 1, 1);
	}

	res = dev->selfops->reg_out(dev->self, UL_GP_ADDR + GP_PORT_WR_LMS_CTRL, powermask);

	if (!((ctrl_mask & XTRXLL_LMS7_GPWR_PIN) || (ctrl_mask & XTRXLL_DCDC_ON))) {
		lp8758_en(dev, 0, 1);
	}

	//uint32_t dac = 0;
	//get_dac_val(dev, &dac);
	//XTRXLL_LOG(XTRXLL_WARNING, "DAC: %08x\n", dac);

	return res;
}

static int xtrvxllv0_get_sensor(struct xtrxll_base_dev* dev, unsigned sensorno, int* outval)
{
	int res;
	uint32_t tmp;

	switch (sensorno) {
	case XTRXLL_PMIC0_VER:
		*outval = 0;
		return lp8758_get(dev->self, 0, DEV_REV, (uint8_t*)outval);
	case XTRXLL_PMIC0_ID:
		*outval = 0;
		return lp8758_get(dev->self, 0, OTP_REV, (uint8_t*)outval);
	case XTRXLL_PMIC1_VER:
		*outval = 0;
		return lp8758_get(dev->self, 1, DEV_REV, (uint8_t*)outval);
	case XTRXLL_PMIC1_ID:
		*outval = 0;
		return lp8758_get(dev->self, 1, OTP_REV, (uint8_t*)outval);

	case XTRXLL_PMIC0_CTRL1:
	case XTRXLL_PMIC1_CTRL1:
	{
#define NUM 4
		uint8_t busno = (sensorno == XTRXLL_PMIC0_CTRL1) ? 0 : 1;
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
		*outval = tmp;
		return res;
	case XTRXLL_REFCLK_CNTR:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_REF_OSC, &tmp);
		*outval = tmp;
		return res;
	case XTRXLL_REFCLK_CLK:
	{
		uint32_t t = 0;
		unsigned i, j;
		unsigned cur, prev = 0;
		*outval = 0;

		for (i = 0, j = 0; i < 8 && j < 2; i++) {
			res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_REF_OSC, &tmp);
			if (res)
				return res;

			cur = (tmp >> 28) & 0xf;
			if (cur == prev) {
				usleep(5000);
				continue;
			}
			prev = cur;
			t += tmp & 0xffffff;
			j++;
		}
		if (j < 2)
			return -ENOENT;

		*outval = (int)(((uint64_t)t * 125000000UL) / 65536 / 16 / j);
		return 0;
	}
	case XTRXLL_ONEPPS_CAPTURED:
		// FIXME!!!!
		//*outval = pread(dev->fd, &reg, sizeof(reg), XTRX_KERN_1PPS_EVENTS);
		//return ( *outval < 0 ) ? *outval : 0;
		return 0;
	case XTRXLL_TEST_CNT_RXIQ_MISS:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_RXIQ_MISS,
								   &tmp);
		*outval = tmp;
		return res;
	case XTRXLL_TEST_CNT_RXIQ_MALGN:
		res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_RXIQ_ODD,
								   &tmp);
		*outval = tmp;
		return res;
	case XTRXLL_HWID:
		*outval = dev->hwid;
		return 0;
	default:
		return -EINVAL;
	}
	return 0;
}

static int xtrvxllv0_get_cfg(struct xtrxll_base_dev* dev, enum xtrxll_cfg param, int* out)
{
	switch (param) {
	case XTRXLL_CFG_NUM_LMS7:        *out = 1; return 0;
	case XTRXLL_CFG_HAS_GPS:         *out = 1; return 0;
	case XTRXLL_CFG_HAS_SIM_READER:  *out = 1; return 0;
	default:     return -EINVAL;
	}
}

static int xtrvxllv0_lms7_ant(struct xtrxll_base_dev* dev, unsigned rx_ant, unsigned tx_ant)
{
	int res = dev->selfops->reg_out(dev->self,
									UL_GP_ADDR + GP_PORT_WR_RF_SWITCHES,
									((tx_ant & 1) << 2) | (rx_ant & 3) );

	XTRXLL_LOG(XTRXLL_INFO,  "XTRX %s: RX_ANT: %d TX_ANT: %d\n",
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

	XTRXLL_LOG(XTRXLL_DEBUG, "XTRX %s: MMCM -> %04x (%04x.%04x.%04x.%04x)\n",
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
		XTRXLL_LOG(XTRXLL_WARNING, "XTRX %s: timmend command queue is full\n", dev->id);
		return -EAGAIN;
	}

	XTRXLL_LOG(XTRXLL_INFO, "XTRX %s: Placing TC @%d on %d data: %x stat:%x\n",
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

static int xtrvxllv0_set_param(struct xtrxll_base_dev* dev, unsigned paramno, unsigned param)
{
	switch (paramno) {
	case XTRXLL_PARAM_CLOCK_TYPE:
	{
		return -EINVAL;
	}
	case XTRXLL_PARAM_PWR_MODE:
		return xtrxllr3_lms7pwr_set_mode(dev, param);
	case XTRXLL_PARAM_PWR_VIO:
		return xtrxllr3_io_set(dev, param);
	case XTRXLL_PARAM_RX_DLY:
		return dev->selfops->reg_out(dev->self,
									 UL_GP_ADDR + GP_PORT_WR_LMS_CTRL,
									 ((param << 17) | (1 <<16)));
	default:
		return -EINVAL;
	}
}

static int xtrvxllv0_set_osc_dac(struct xtrxll_base_dev* dev, unsigned val)
{
	if (GET_FWID(dev->hwid) == FWID_XTRX_R4)
		return mcp4725_set_cur(dev, val);

	return ltc2606_set_cur(dev, val);
}

const static struct xtrxll_ctrl_ops s_xtrx_base_ops = {
	.get_cfg = xtrvxllv0_get_cfg,

	.lms7_pwr_ctrl = xtrvxllv0_lms7_pwr_ctrl,
	.lms7_ant = xtrvxllv0_lms7_ant,
	.get_sensor = xtrvxllv0_get_sensor,

	.set_osc_dac = xtrvxllv0_set_osc_dac,
	.get_osc_freq = xtrvxllv0_get_osc_freq,

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

	int res = dev->selfops->reg_in(dev->self, UL_GP_ADDR + GP_PORT_RD_HWCFG,
											  &dev->hwid);
	if (res)
		return res;

	switch (GET_FWID(dev->hwid)) {
	case FWID_XTRX_R3:
		XTRXLL_LOG(XTRXLL_INFO, "XTRX %s: XTRX Rev3 (%08x)\n", dev->id, dev->hwid);
		return 0;
	case FWID_XTRX_R4:
		XTRXLL_LOG(XTRXLL_INFO, "XTRX %s: XTRX Rev4 (%08x)\n", dev->id, dev->hwid);
		return 0;
	}

	XTRXLL_LOG(XTRXLL_ERROR, "XTRX %s: Unrecognized HWID %08x!\n", dev->id, dev->hwid);
	return -ENOTSUP;
}
