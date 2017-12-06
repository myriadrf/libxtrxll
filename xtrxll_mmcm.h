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

enum {
	XTRXLL_MMCM_ON     = 1,
	XTRXLL_MMCM_MCLK2  = 2,
};
int xtrxll_mmcm_setfreq(struct xtrxll_dev* dev, int mclk, int rxx2, int txx2,
						int rx_fwd_delay);
int xtrxll_mmcm_onoff(struct xtrxll_dev* dev, int on);

#endif
