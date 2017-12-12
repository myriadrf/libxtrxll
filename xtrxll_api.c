/*
 * Public xtrx low level API source file
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
#ifdef __linux
#define _GNU_SOURCE
#include <dlfcn.h>
#endif
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdbool.h>

#include "xtrxll_log.h"
#include "xtrxll_api.h"
#include "xtrxll_base.h"

struct xtrxll_driver
{
	struct xtrxll_driver* next;

	/** module operations */
	const struct xtrxll_ops* module_ops;
	int *module_loglevel;

	void *dyn_handle;
};

struct xtrxll_dev_loader
{
	struct xtrxll_base_dev base;

	struct xtrxll_base_dev* inner;
	void* handle;
};

/* TODO: do dynamic enumeration; just for now use predefined modules */
static const char* s_known_modlibs[] = {
#ifdef __linux
	"libxtrxll_pcie.so.0",
	"libxtrxll_libusb3380.so.0",
	"libxtrxll_nativeusb.so.0"
#elif defined(_WIN32) || defined(_WIN32_WCE)
	"libxtrxll_pcie.dll",
	"libxtrxll_libusb3380.dll",
	"libxtrxll_nativeusb.dll"
#elif defined(__APPLE__)
	"libxtrxll_libusb3380.dylib",
	"libxtrxll_nativeusb.dylib"
#endif
};


int xtrxll_init_drivers(unsigned flags)
{
	// Initialize dynamic modules
	const char* ld_lib;
	char *error;
	int *module_loglevel;
	xtrxll_init_func_t func;
	const struct xtrxll_ops* selfops;
	void* handle;
	unsigned i;

	struct xtrxll_driver* drvs = NULL;
	struct xtrxll_driver* drv_new;

	xtrxll_log_initialize(NULL);

	// Initialize static modules

	for (i = 0; i < sizeof(s_known_modlibs) / sizeof(s_known_modlibs[0]); i++) {
		ld_lib = s_known_modlibs[i];
		XTRXLL_LOG(XTRXLL_INFO, "Probing '%s' low-level library\n", ld_lib);

		//handle = dlmopen(LM_ID_NEWLM, ld_lib, RTLD_NOW | RTLD_LOCAL);
		handle = dlopen(ld_lib, RTLD_NOW | RTLD_LOCAL);
		if (!handle) {
			XTRXLL_LOG(XTRXLL_ERROR, "Error loading: %s\n", dlerror());
			continue;
		}

		dlerror();
		func = (xtrxll_init_func_t)dlsym(handle, XTRXLL_INIT_FUNC);
		error = dlerror();
		if (error != NULL) {
			XTRXLL_LOG(XTRXLL_ERROR, "Can't locate symbol: %s\n", error);
			dlclose(handle);
			continue;
		}

		selfops = func(XTRXLL_ABI_VERSION);
		if (selfops == NULL) {
			XTRXLL_LOG(XTRXLL_WARNING, "Library '%s' ABI mismatch\n", ld_lib);
			dlclose(handle);
			continue;
		}

		/* loglevel is hidden inside the namespace, so we need to update to the
		 * actual value */
		module_loglevel = (int*)dlsym(handle, "s_loglevel");
		if (module_loglevel) {
			XTRXLL_LOG(XTRXLL_DEBUG, "low-level s_loglevel: %p\n", module_loglevel);
			*module_loglevel = s_loglevel;
		}

		drv_new = (struct xtrxll_driver *)malloc(sizeof(struct xtrxll_driver));
		if (drv_new == NULL) {
			dlclose(handle);
			return -ENOMEM;
		}

		drv_new->next = drvs;
		drv_new->module_ops = selfops;
		drv_new->module_loglevel = module_loglevel;
		drv_new->dyn_handle = handle;

		drvs = drv_new;
	}

	return 0;
}


int xtrxll_open(const char* device, unsigned flags, struct xtrxll_dev** odev)
{
	const char* ld_lib;
	int res;
	char *error;
	int *module_loglevel;
	xtrxll_init_func_t func;
	const struct xtrxll_ops* selfops;
	struct xtrxll_base_dev* under_dev;
	struct xtrxll_dev_loader* dev;
	void* handle;

	xtrxll_log_initialize(NULL);

	for (unsigned i = 0; i < sizeof(s_known_modlibs) / sizeof(s_known_modlibs[0]); i++) {
		ld_lib = s_known_modlibs[i];
		XTRXLL_LOG(XTRXLL_INFO, "Probing '%s' low-level library\n", ld_lib);

		//handle = dlmopen(LM_ID_NEWLM, ld_lib, RTLD_NOW | RTLD_LOCAL);
		handle = dlopen(ld_lib, RTLD_NOW | RTLD_LOCAL);
		if (!handle) {
			XTRXLL_LOG(XTRXLL_ERROR, "Error loading: %s\n", dlerror());
			continue;
		}

		dlerror();
		func = (xtrxll_init_func_t)dlsym(handle, XTRXLL_INIT_FUNC);
		error = dlerror();
		if (error != NULL) {
			XTRXLL_LOG(XTRXLL_ERROR, "Can't locate symbol: %s\n", error);
			dlclose(handle);
			continue;
		}

		selfops = func(XTRXLL_ABI_VERSION);
		if (selfops == NULL) {
			XTRXLL_LOG(XTRXLL_WARNING, "Library '%s' ABI mismatch\n", ld_lib);
			dlclose(handle);
			continue;
		}

		/* loglevel is hidden inside the namespace, so we need to update to the
		 * actual value */
		module_loglevel = (int*)dlsym(handle, "s_loglevel");
		if (module_loglevel) {
			XTRXLL_LOG(XTRXLL_DEBUG, "low-level s_loglevel: %p\n", module_loglevel);
			*module_loglevel = s_loglevel;
		}

		res = selfops->open(device, flags, &under_dev);
		if (res) {
			dlclose(handle);
			continue;
		}

		dev = (struct xtrxll_dev_loader*)malloc(sizeof(struct xtrxll_dev_loader));
		if (dev == NULL) {
			selfops->close(under_dev);
			dlclose(handle);
			return -ENOMEM;
		}

		dev->base = *under_dev;
		dev->inner = under_dev;
		dev->handle = handle;

		*odev = (struct xtrxll_dev*)dev;
		return 0;
	}

	return -ENODEV;
}

int xtrxll_discovery(char* devices, size_t maxbuf)
{
	return -1;
}

void xtrxll_close(struct xtrxll_dev* dev)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	ldev->base.selfops->close(ldev->base.self);

#ifdef __linux
	dlclose(ldev->handle);
#endif

	free(ldev);
}

const char* xtrxll_get_name(struct xtrxll_dev* dev)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.id;
}

int xtrxll_get_cfg(struct xtrxll_dev* dev, enum xtrxll_cfg param, int* out)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.ctrlops->get_cfg(ldev->base.self, param, out);
}

int xtrxll_lms7_spi_bulk(struct xtrxll_dev* dev, uint32_t lmsno,
						 const uint32_t* out, uint32_t* in, size_t count)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->spi_bulk(ldev->inner, lmsno, out, in, count);
}


int xtrxll_lms7_pwr_ctrl(struct xtrxll_dev* dev, uint32_t lmsno,
						 unsigned ctrl_mask)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.ctrlops->lms7_pwr_ctrl(ldev->inner, lmsno, ctrl_mask);
}

int xtrxll_lms7_ant(struct xtrxll_dev* dev, unsigned rx_ant, unsigned tx_ant)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.ctrlops->lms7_ant(ldev->inner, rx_ant, tx_ant);
}

int xtrxll_get_sensor(struct xtrxll_dev* dev, unsigned sensorno, int* outval)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->get_sensor(ldev->inner, sensorno, outval);
}

int xtrxll_set_param(struct xtrxll_dev* dev, unsigned paramno, unsigned value)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->set_param(ldev->inner, paramno, value);
}

int xtrxll_dma_rx_init(struct xtrxll_dev* dev, int chan, unsigned buf_szs,
					   unsigned* out_szs)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->dma_rx_init(ldev->inner, chan, buf_szs, out_szs);
}

int xtrxll_dma_rx_deinit(struct xtrxll_dev* dev, int chan)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->dma_rx_deinit(ldev->inner, chan);
}

int xtrxll_dma_rx_getnext(struct xtrxll_dev* dev, int chan, void** addr,
						  wts_long_t *ts, unsigned *sz, unsigned flags)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->dma_rx_getnext(ldev->inner, chan, addr,
											  ts, sz, flags);
}

int xtrxll_dma_rx_release(struct xtrxll_dev* dev, int chan, void* addr)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->dma_rx_release(ldev->inner, chan, addr);
}

int xtrxll_dma_rx_resume_at(struct xtrxll_dev* dev, int chan, wts_long_t nxt)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->dma_rx_resume_at(ldev->inner, chan, nxt);
}

int xtrxll_dma_tx_init(struct xtrxll_dev* dev, int chan, unsigned buf_szs)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->dma_tx_init(ldev->inner, chan, buf_szs);
}

int xtrxll_dma_tx_deinit(struct xtrxll_dev* dev, int chan)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->dma_tx_deinit(ldev->inner, chan);
}

int xtrxll_dma_tx_getfree_ex(struct xtrxll_dev* dev, int chan, void** addr,
							 uint16_t* late)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->dma_tx_getfree_ex(ldev->inner, chan, addr, late);
}

int xtrxll_dma_tx_post(struct xtrxll_dev* dev, int chan, void* addr,
					   wts_long_t wts, uint32_t samples)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->dma_tx_post(ldev->inner, chan, addr, wts,
										   samples);
}

int xtrxll_dma_start(struct xtrxll_dev* dev, int chan,
					 xtrxll_fe_t rxfe, xtrxll_mode_t rxmode,
					 wts_long_t rx_start_sample,
					 xtrxll_fe_t txfe, xtrxll_mode_t txmode)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->dma_start(ldev->inner, chan, rxfe, rxmode,
										 rx_start_sample, txfe, txmode);
}

int xtrxll_set_osc_dac(struct xtrxll_dev* dev, unsigned val)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.ctrlops->set_osc_dac(ldev->inner, val);
}

int xtrxll_get_osc_freq(struct xtrxll_dev* dev, uint32_t *regval)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.ctrlops->get_osc_freq(ldev->inner, regval);
}


int xtrxll_set_txmmcm(struct xtrxll_dev* dev, uint16_t reg, uint16_t value)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.ctrlops->set_txmmcm(ldev->inner, reg, value);
}

int xtrxll_get_txmmcm(struct xtrxll_dev* dev, uint16_t* value,
					  uint8_t* locked, uint8_t* rdy)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.ctrlops->get_txmmcm(ldev->inner, value, locked, rdy);
}

int xtrxll_repeat_tx_buf(struct xtrxll_dev* dev, int chan, xtrxll_fe_t fmt,
						 const void* buff, unsigned buf_szs, xtrxll_mode_t mode)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->repeat_tx_buf(ldev->inner, chan, fmt, buff,
											 buf_szs, mode);
}

int xtrxll_repeat_tx_start(struct xtrxll_dev* dev, int chan, int start)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.selfops->repeat_tx_start(ldev->inner, chan, start);
}

int xtrxll_read_uart(struct xtrxll_dev* dev, unsigned uartno,
					 uint8_t* out, unsigned maxsize, unsigned *written)
{
	struct xtrxll_dev_loader* ldev = (struct xtrxll_dev_loader*)dev;
	return ldev->base.ctrlops->read_uart(ldev->inner, uartno, out, maxsize,
										 written);
}

