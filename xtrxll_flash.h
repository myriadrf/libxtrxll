/*
 * Public flash API header file
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
#ifndef XTRXLL_FLASH_H
#define XTRXLL_FLASH_H

#include "xtrxll_api.h"
#include "xtrxll_base.h"

/** @file xtrxll_flash.h Low level flash access
 *
 * This file provide interface for low-level flash access, such as block read or
 * block write. No special checks are made about the safety of the region, if
 * incorrect offset were used it can corrupt FPGA image. For user data access
 * use high level API
 */

/**
 * @brief xtrxll_flash_get_id Read basic flash identifier (first 4 bytes)
 * @param dev
 * @param flash_id
 * @param capacity Capacity in bytes, 0 - means unknown flash type
 * @return 0 on success, errno otherwise
 */
int xtrxll_flash_get_id(struct xtrxll_base_dev* dev, uint32_t *flash_id,
						uint32_t *capacity, char* outid, size_t maxstr);

/**
 * @brief xtrxll_flash_to_host Copy data from FLASH memory on XTRX to the host
 * @param dev Device
 * @param flash_off Offset in bytes on Flash
 * @param size Total transfer size
 * @param out Pointer to host memory
 * @return 0 on success, errno otherwise
 */
int xtrxll_flash_to_host(struct xtrxll_base_dev *dev, uint32_t flash_off,
						 uint32_t size, char* out);

/**
 * @brief xtrxll_flash_from_host Erase secotrs and copy data to flash
 * @param dev
 * @param out
 * @param size
 * @param flash_off
 * @return
 */
int xtrxll_flash_from_host(struct xtrxll_base_dev *dev, const char* out,
						   uint32_t size, uint32_t flash_off,
						   unsigned erase_flags);

#endif
