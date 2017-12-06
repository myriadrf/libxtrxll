/*
 * Public flash source file
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
#include "xtrxll_flash.h"
#include "xtrxll_log.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>

/* Include verilog file used for FPGA build */
#define localparam static const unsigned
#include "xtrxll_regs.vh"

localparam QIO_MIORDID  = 0xAF;  //    0         0          1-3             0
localparam QIO_RDSFDP   = 0x5A;  //    3         8          1+              0
localparam QIO_QCFR_0   = 0x0B;  //    3/4       10         1+              0
localparam QIO_QCFR_1   = 0x6B;  //    3/4       10         1+              0
localparam QIO_QCFR_2   = 0xEB;  //    3/4       10         1+              0
localparam QIO_QCFR4B_0 = 0x0C;  //    4         10         1+              0
localparam QIO_QCFR4B_1 = 0x6C;  //    4         10         1+              0
localparam QIO_QCFR4B_2 = 0xEC;  //    4         10         1+              0
localparam QIO_ROTP     = 0x4B;  //    3/4       10         1-65            0

localparam QIO_WREN     = 0x06;  //    0         0          0               0
localparam QIO_WRDI     = 0x04;  //    0         0          0               0

// Program commands
localparam QIO_QCPP_0   = 0x02;  //    3/4       0          0               1-256
localparam QIO_QCPP_1   = 0x32;  //    3/4       0          0               1-256
localparam QIO_QCPP_2   = 0x12;  //    3/4       0          0               1-256
localparam QIO_POTP     = 0x22;  //    3/4       0          0               1-65

localparam QIO_SSE      = 0x20;  //    3/4       0          0               0
localparam QIO_SE       = 0xD8;  //    3/4       0          0               0
localparam QIO_BE       = 0xC7;  //    0         0          0               0
localparam QIO_PER      = 0x7A;  //    0         0          0               0
localparam QIO_PES      = 0x75;  //    0         0          0               0

localparam QIO_RDSR     = 0x05;  //    0         0          1+              0
localparam QIO_WRSR     = 0x01;  //    0         0          0               1
localparam QIO_RDLR     = 0xE8;  //    3/4       0          1+              0
localparam QIO_WRLR     = 0xE5;  //    3/4       0          0               1
localparam QIO_RFSR     = 0x70;  //    0         0          2               0
localparam QIO_CLFSR    = 0x50;  //    0         0          0               2
localparam QIO_RDVCR    = 0x85;  //    0         0          1+              0
localparam QIO_WRVCR    = 0x81;  //    0         0          0               1
localparam QIO_RDECR    = 0x65;  //    0         0          1+              0
localparam QIO_WRECR    = 0x61;  //    0         0          0               1

localparam QIO_EN4BYTEA = 0xB7;  //    0         0          0               0
localparam QIO_EX4BYTEA = 0xE9;  //    0         0          0               0
localparam QIO_WREAR    = 0xC5;  //    0         0          0               0
localparam QIO_RDREAR   = 0xC8;  //    0         0          0               0

localparam QIO_RSTEN    = 0x66;  //    0         0          0               0
localparam QIO_RST      = 0x99;  //    0         0          0               0


localparam ESPI_RDID_0  = 0x9E;
localparam ESPI_RDID_1  = 0x9F;

// QIO READ, only first 8b are 1bit
//localparam ESPI_QIOFR   = QIO_QCFR_2;
//localparam ESPI_QIOFR4B = QIO_QCFR4B_2;

// QIO PROGRAMM
//localparam ESPI_QIEFP   = QIO_QCPP_2;

localparam FLASH_CMD_OFF = 24;
localparam FLASH_SIZE_OFF = 16;
localparam FLASH_RAM_16B_ADDR_OFF = 4;
localparam FLASH_RAM_EN_OFF = 3;
localparam FLASH_ADDR_SZ_OFF = 1;
localparam FLASH_WRITE_OP_OFF = 0;

// FIXME: only 1 page works for now
localparam FLASH_MEM_TOP = 0;

// #define DEBUG_CHECK_XTRX_RAM

static int xtrxll_reg_out(struct xtrxll_base_dev* dev, uint32_t reg, uint32_t val)
{
	return dev->selfops->reg_out(dev->self, reg, val);
}

static int xtrxll_reg_in(struct xtrxll_base_dev* dev, uint32_t reg, uint32_t* val)
{
	return dev->selfops->reg_in(dev->self, reg, val);
}

static int flash_wait_done(struct xtrxll_base_dev* dev)
{
	unsigned limit = 0;
	uint32_t fst;
	do {
		int res = xtrxll_reg_in(dev, GP_PORT_RD_QSPI_STAT, &fst);
		if (res)
			return res;

		if (limit > 1000000)
			return -ETIMEDOUT;

		usleep(1);
		limit++;
	} while ((fst & 1) != 0);

	return 0;
}

static int flash_read_reg(struct xtrxll_base_dev* dev, uint8_t reg,
						  uint8_t sz, uint32_t* out)
{
	int res;
	res = xtrxll_reg_out(dev, GP_PORT_WR_QSPI_CMD,
						 ((uint32_t)reg << FLASH_CMD_OFF) |
						 ((uint32_t)sz << FLASH_SIZE_OFF) |
						 (0 << FLASH_RAM_16B_ADDR_OFF) |
						 (0 << FLASH_RAM_EN_OFF) |
						 (0 << FLASH_ADDR_SZ_OFF) |
						 (0 << FLASH_WRITE_OP_OFF));
	if (res)
		return res;

	res = flash_wait_done(dev);
	if (res)
		return res;

	res = xtrxll_reg_in(dev, GP_PORT_RD_QSPI_RB, out);
	if (res)
		return res;

	return 0;
}

#define JEDEC_MICRON          0x20
#define MICRON_SERIAL_NOR     0xBA
#define MICRON_SERIAL_NOR_18  0xBB

int xtrxll_flash_get_id(struct xtrxll_base_dev *dev, uint32_t *flash_id,
						uint32_t *capacity, char *outid, size_t maxstr)
{
	if (capacity)
		*capacity = 0;
	if (outid)
		*outid = 0;

	uint32_t id;
	int res = flash_read_reg(dev, ESPI_RDID_1, 4, &id);
	if (res)
		return res;

	*flash_id = id;
	uint8_t manufacture_id = id & 0xff;
	uint8_t flash_type = ((id >> 8) & 0xff);
	uint8_t flash_size = ((id >> 16) & 0xff);

	uint32_t cap = 1U << flash_size;

#if 1
	uint32_t volatile_cfg, volatile_excfg;
	res = flash_read_reg(dev, QIO_RDVCR, 4, &volatile_cfg);
	if (res)
		return res;

	res = flash_read_reg(dev, QIO_RDECR, 4, &volatile_excfg);
	if (res)
		return res;
#endif

	if (manufacture_id == JEDEC_MICRON) {
		if (flash_type == MICRON_SERIAL_NOR || flash_type == MICRON_SERIAL_NOR_18) {
			snprintf(outid, maxstr, "Micron Serial NOR MT25Q %d Mb (%s) [%02x %02x]",
					 8 * cap / 1024 / 1024,
					 (flash_type == MICRON_SERIAL_NOR) ? "3.3V" : "1.8V",
					 volatile_cfg, volatile_excfg);
		}
	}

	if (capacity)
		*capacity = cap;

	return res;
}

static int flash_rdsr(struct xtrxll_base_dev* dev, uint8_t* out)
{
	uint32_t v;
	int res = flash_read_reg(dev, QIO_RDSR, 1, &v);
	if (res)
		return res;

	*out = (uint8_t)v;
	return 0;
}

#if 0
static int flash_rfsr(struct xtrxll_base_dev* dev, uint8_t* out)
{
	uint32_t v;
	int res = flash_read_reg(dev, QIO_RFSR, 1, &v);
	if (res)
		return res;

	*out = v;
	return 0;
}

static int flash_rdvcr(struct xtrxll_base_dev* dev, uint8_t* out)
{
	uint32_t v;
	int res = flash_read_reg(dev, QIO_RDVCR, 1, &v);
	if (res)
		return res;

	*out = v;
	return 0;
}
#endif

static int flash_read(struct xtrxll_base_dev* dev, uint8_t cmd, uint8_t sz,
					  uint32_t flash_addr, uint32_t* out)
{
	int res;
	res = xtrxll_reg_out(dev, GP_PORT_WR_QSPI_EXCMD, flash_addr);
	if (res)
		return res;

	res = xtrxll_reg_out(dev, GP_PORT_WR_QSPI_CMD,
						 ((uint32_t)cmd << FLASH_CMD_OFF) |
						 ((uint32_t)sz << FLASH_SIZE_OFF) |
						 ((FLASH_MEM_TOP >> 4) << FLASH_RAM_16B_ADDR_OFF) |
						 (1 << FLASH_RAM_EN_OFF) |
						 (2 << FLASH_ADDR_SZ_OFF) |
						 (0 << FLASH_WRITE_OP_OFF));
	if (res)
		return res;

	res = flash_wait_done(dev);
	if (res)
		return res;

	res = dev->ctrlops->mem_rb32(dev, FLASH_MEM_TOP / 4,
								 (sz == 0) ? (256 / 4) : ((sz + 3) / 4), out);
	return res;
}

int xtrxll_flash_to_host(struct xtrxll_base_dev* dev, uint32_t flash_off,
						 uint32_t size, char* out)
{
	int res = -EINVAL;
	uint32_t addr = flash_off;
	while (size > 0) {
		uint32_t bsz = (size > 256) ? 256 : size;

		res = flash_read(dev, QIO_QCFR_2, bsz, addr, (uint32_t*)out);
		if (res != bsz / 4)
			return res;

		out += bsz;
		size -= bsz;
		addr += bsz;
	}

	return 0;
}


static int flash_wren(struct xtrxll_base_dev* dev)
{
	return xtrxll_reg_out(dev, GP_PORT_WR_QSPI_CMD,
						  (QIO_WREN << FLASH_CMD_OFF) |
						  (0 << FLASH_SIZE_OFF) |
						  (0 << FLASH_RAM_16B_ADDR_OFF) |
						  (0 << FLASH_RAM_EN_OFF) |
						  (0 << FLASH_ADDR_SZ_OFF) |
						  (0 << FLASH_WRITE_OP_OFF));
}

static int flash_bulk_erase(struct xtrxll_base_dev* dev)
{
	return xtrxll_reg_out(dev, GP_PORT_WR_QSPI_CMD,
						  (QIO_BE << FLASH_CMD_OFF) |
						  (0 << FLASH_SIZE_OFF) |
						  (0 << FLASH_RAM_16B_ADDR_OFF) |
						  (0 << FLASH_RAM_EN_OFF) |
						  (0 << FLASH_ADDR_SZ_OFF) |
						  (0 << FLASH_WRITE_OP_OFF));
}

static int flash_subsector_erase(struct xtrxll_base_dev* dev, uint32_t addr)
{
	int res = xtrxll_reg_out(dev, GP_PORT_WR_QSPI_EXCMD, addr);
	if (res)
		return res;

	return xtrxll_reg_out(dev, GP_PORT_WR_QSPI_CMD,
						  (QIO_SSE << FLASH_CMD_OFF) |
						  (0 << FLASH_SIZE_OFF) |
						  (0 << FLASH_RAM_16B_ADDR_OFF) |
						  (0 << FLASH_RAM_EN_OFF) |
						  (2 << FLASH_ADDR_SZ_OFF) |
						  (0 << FLASH_WRITE_OP_OFF));
}

static int flash_sector_erase(struct xtrxll_base_dev* dev, uint32_t addr)
{
	int res = xtrxll_reg_out(dev, GP_PORT_WR_QSPI_EXCMD, addr);
	if (res)
		return res;

	return xtrxll_reg_out(dev, GP_PORT_WR_QSPI_CMD,
						  (QIO_SE << FLASH_CMD_OFF) |
						  (0 << FLASH_SIZE_OFF) |
						  (0 << FLASH_RAM_16B_ADDR_OFF) |
						  (0 << FLASH_RAM_EN_OFF) |
						  (2 << FLASH_ADDR_SZ_OFF) |
						  (0 << FLASH_WRITE_OP_OFF));
}

#define WATCHDOG_LIMIT 100000
#define RDSR_BUSY_BIT (1)

static int flash_wait_ready(struct xtrxll_base_dev* dev)
{
	int res;
	uint8_t status;
	unsigned limit = 0;

	// Wait for finalizing
	do {
		res = flash_rdsr(dev, &status);
		if (res)
			return res;

		if ((status & RDSR_BUSY_BIT) != RDSR_BUSY_BIT)
			break;

		usleep(1);
		limit++;
	} while (limit < WATCHDOG_LIMIT);

	if (limit == WATCHDOG_LIMIT)
		return -ETIMEDOUT;

#if 0
	flash_rfsr(dev, &status);
	fprintf(stderr, "RFSF=%x\n", status);
#endif
	return 0;
}

static int flash_erase(struct xtrxll_base_dev* dev, uint32_t addr, uint32_t size)
{
	const unsigned subsec_msk = 0x0fff; // 4kb
	const unsigned sec_msk    = 0xffff; // 64kb
	int res;

	if (size <= subsec_msk)
		return -EINVAL;

	do {
		if (addr == 0 && size == ~((uint32_t)0)) {
			res = flash_wren(dev);
			if (res)
				return res;

			res = flash_bulk_erase(dev);
			if (res)
				return res;

			size = 0;
		} else if (((addr & sec_msk) == 0) && (size > sec_msk)) {
			res = flash_wren(dev);
			if (res)
				return res;

			//Sector ERASE
			res = flash_sector_erase(dev, addr);
			if (res)
				return res;

			addr += sec_msk + 1;
			size -= sec_msk + 1;
		} else if (((addr & subsec_msk) == 0) && (size > subsec_msk)) {
			res = flash_wren(dev);
			if (res)
				return res;

			//Sub sector ERASE
			res = flash_subsector_erase(dev, addr);
			if (res)
				return res;

			addr += subsec_msk + 1;
			size -= subsec_msk + 1;
		} else {
			// Granularity less than subsector
			return -EINVAL;
		}

		res = flash_wait_ready(dev);
		if (res)
			return res;

	} while (size != 0);

	return 0;
}

static int flash_write(struct xtrxll_base_dev* dev, uint8_t cmd, uint8_t sz,
					   uint32_t flash_addr, const uint32_t* in)
{
	int res;
	res = dev->ctrlops->mem_wr32(dev->self, FLASH_MEM_TOP / 4,
								 (sz == 0) ? (256 / 4) : ((sz + 3) / 4), in);
	if (res)
		return res;

#ifdef DEBUG_CHECK_XTRX_RAM
	uint32_t tmp[64];
	xtrxll_mem_rb(dev, FLASH_MEM_TOP / 4,
				  (sz == 0) ? (256 / 4) : ((sz + 3) / 4), tmp);

	res = memcmp(tmp, in, (sz == 0) ? 256 : sz);
	assert (res == 0);
#endif

	res = flash_wren(dev);
	if (res)
		return res;

	res = xtrxll_reg_out(dev, GP_PORT_WR_QSPI_EXCMD, flash_addr);
	if (res)
		return res;

	res = xtrxll_reg_out(dev, GP_PORT_WR_QSPI_CMD,
						 ((uint32_t)cmd << FLASH_CMD_OFF) |
						 ((uint32_t)sz << FLASH_SIZE_OFF) |
						 ((FLASH_MEM_TOP >> 4) << FLASH_RAM_16B_ADDR_OFF) |
						 (1 << FLASH_RAM_EN_OFF) |
						 (2 << FLASH_ADDR_SZ_OFF) |
						 (1 << FLASH_WRITE_OP_OFF));
	if (res)
		return res;

	usleep(1000);

	res = flash_wait_done(dev);
	if (res)
		return res;

	res = flash_wait_ready(dev);
	return res;
}

int xtrxll_flash_from_host(struct xtrxll_base_dev* dev, const char* in,
						   uint32_t size, uint32_t flash_off,
						   unsigned erase_flags)
{
	(void)erase_flags;

	int res;
	res = flash_erase(dev, flash_off, size);
	if (res)
		return res;

	uint32_t addr = flash_off;
	while (size > 0) {
		uint32_t bsz = (size > 256) ? 256 : size;

		res = flash_write(dev, QIO_QCPP_0, bsz, addr, (const uint32_t*)in);
		if (res)
			return res;

		in += bsz;
		size -= bsz;
		addr += bsz;
	}

	return res;
}
