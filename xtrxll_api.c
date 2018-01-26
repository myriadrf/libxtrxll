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
	/** dynamic or compiled-in plugin */
	void *dyn_handle;
	const char *proto_id;

	bool dynamic;
};

/* TODO: do dynamic enumeration; just for now use predefined modules */
static const char* s_known_modlibs[] = {
#ifdef __linux
	"libxtrxll_pcie.so.0",
	"libxtrxll_libusb3380.so.0",
//	"libxtrxll_nativeusb.so.0",
#elif defined(_WIN32) || defined(_WIN32_WCE)
	"libxtrxll_pcie.dll",
	"libxtrxll_libusb3380.dll",
	"libxtrxll_nativeusb.dll",
#elif defined(__APPLE__)
	"libxtrxll_libusb3380.dylib",
	"libxtrxll_nativeusb.dylib",
#endif
};

static struct xtrxll_driver* s_api_enumerated_modules = NULL;

static void api_drivers_clean_recurscion(struct xtrxll_driver* drv)
{
	if (drv->next) {
		api_drivers_clean_recurscion(drv->next);
	}
	if (drv->dynamic) {
		dlclose(drv->dyn_handle);
	}
	free(drv);
}

static void api_drivers_clean(void)
{
	if (s_api_enumerated_modules) {
		api_drivers_clean_recurscion(s_api_enumerated_modules);
		s_api_enumerated_modules = NULL;
	}
}

static int api_drivers_init(void)
{
	// Initialize dynamic modules
	const char* ld_lib;
	char *error;
	xtrxll_init_func_t func;
	const struct xtrxll_ops* selfops;
	void* handle;
	unsigned i;

	struct xtrxll_driver* drvs = NULL;
	struct xtrxll_driver* drv_new;

	xtrxll_log_initialize(NULL);

	// TODO initialize compiled-in modules

	// Initialize static modules
	for (i = 0; i < sizeof(s_known_modlibs) / sizeof(s_known_modlibs[0]); i++) {
		ld_lib = s_known_modlibs[i];
		XTRXLL_LOG(XTRXLL_INFO, "Probing '%s' low-level library\n", ld_lib);

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

		drv_new = (struct xtrxll_driver *)malloc(sizeof(struct xtrxll_driver));
		if (drv_new == NULL) {
			dlclose(handle);

			// TODO list cleanup
			return -ENOMEM;
		}

		drv_new->next = drvs;
		drv_new->module_ops = selfops;
		drv_new->dynamic = true;
		drv_new->dyn_handle = handle;

		drv_new->proto_id = drv_new->module_ops->get_proto_id();

		drvs = drv_new;
	}

	s_api_enumerated_modules = drvs;
	return 0;
}

static int api_drivers_check_initialized(void)
{
	if (s_api_enumerated_modules == NULL) {
		return api_drivers_init();
	}
	return 0;
}


int xtrxll_open(const char* device, unsigned flags, struct xtrxll_dev** odev)
{
	int res;
	res = api_drivers_check_initialized();
	if (res)
		return res;

	struct xtrxll_driver* nxt = s_api_enumerated_modules;
	struct xtrxll_base_dev* under_dev;

	for (; nxt != NULL; nxt = nxt->next) {
		res = nxt->module_ops->open(device, flags, &under_dev);
		if (res) {
			continue;
		}

		*odev = (struct xtrxll_dev*)under_dev;
		return 0;
	}

	return -ENODEV;
}

int xtrxll_discovery(xtrxll_device_info_t *buffer, size_t maxbuf)
{
	int res;
	res = api_drivers_check_initialized();
	if (res)
		return res;

	struct xtrxll_driver* nxt = s_api_enumerated_modules;
	size_t remaining = maxbuf;
	xtrxll_device_info_t* ptr = buffer;
	for (; nxt != NULL && remaining != 0; nxt = nxt->next) {
		int count = nxt->module_ops->discovery(ptr, remaining);
		if (count > 0) {
			ptr += count;
			remaining -= count;
		}
	}

	return maxbuf - remaining;
}

void xtrxll_close(struct xtrxll_dev* dev)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->close(bdev->self);
}

const char* xtrxll_get_name(struct xtrxll_dev* dev)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->id;
}

int xtrxll_get_cfg(struct xtrxll_dev* dev, enum xtrxll_cfg param, int* out)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->ctrlops->get_cfg(bdev->self, param, out);
}

int xtrxll_lms7_spi_bulk(struct xtrxll_dev* dev, uint32_t lmsno,
						 const uint32_t* out, uint32_t* in, size_t count)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->spi_bulk(bdev->self, lmsno, out, in, count);
}


int xtrxll_lms7_pwr_ctrl(struct xtrxll_dev* dev, uint32_t lmsno,
						 unsigned ctrl_mask)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->ctrlops->lms7_pwr_ctrl(bdev->self, lmsno, ctrl_mask);
}

int xtrxll_lms7_ant(struct xtrxll_dev* dev, unsigned rx_ant, unsigned tx_ant)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->ctrlops->lms7_ant(bdev->self, rx_ant, tx_ant);
}

int xtrxll_get_sensor(struct xtrxll_dev* dev, unsigned sensorno, int* outval)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->get_sensor(bdev->self, sensorno, outval);
}

int xtrxll_set_param(struct xtrxll_dev* dev, unsigned paramno, unsigned value)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->set_param(bdev->self, paramno, value);
}

int xtrxll_dma_rx_init(struct xtrxll_dev* dev, int chan, unsigned buf_szs,
					   unsigned* out_szs)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->dma_rx_init(bdev->self, chan, buf_szs, out_szs);
}

int xtrxll_dma_rx_deinit(struct xtrxll_dev* dev, int chan)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->dma_rx_deinit(bdev->self, chan);
}

int xtrxll_dma_rx_getnext(struct xtrxll_dev* dev, int chan, void** addr,
						  wts_long_t *ts, unsigned *sz, unsigned flags,
						  unsigned timeout_ms)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->dma_rx_getnext(bdev->self, chan, addr,
										 ts, sz, flags, timeout_ms);
}

int xtrxll_dma_rx_release(struct xtrxll_dev* dev, int chan, void* addr)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->dma_rx_release(bdev->self, chan, addr);
}

int xtrxll_dma_rx_resume_at(struct xtrxll_dev* dev, int chan, wts_long_t nxt)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->dma_rx_resume_at(bdev->self, chan, nxt);
}

int xtrxll_dma_tx_init(struct xtrxll_dev* dev, int chan, unsigned buf_szs)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->dma_tx_init(bdev->self, chan, buf_szs);
}

int xtrxll_dma_tx_deinit(struct xtrxll_dev* dev, int chan)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->dma_tx_deinit(bdev->self, chan);
}

int xtrxll_dma_tx_getfree_ex(struct xtrxll_dev* dev, int chan, void** addr,
							 uint16_t* late, unsigned timeout_ms)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->dma_tx_getfree_ex(bdev->self, chan, addr, late,
											timeout_ms);
}

int xtrxll_dma_tx_post(struct xtrxll_dev* dev, int chan, void* addr,
					   wts_long_t wts, uint32_t samples)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->dma_tx_post(bdev->self, chan, addr, wts,
										   samples);
}

int xtrxll_dma_start(struct xtrxll_dev* dev, int chan,
					 xtrxll_fe_t rxfe, xtrxll_mode_t rxmode,
					 wts_long_t rx_start_sample,
					 xtrxll_fe_t txfe, xtrxll_mode_t txmode)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->dma_start(bdev->self, chan, rxfe, rxmode,
										 rx_start_sample, txfe, txmode);
}

int xtrxll_set_osc_dac(struct xtrxll_dev* dev, unsigned val)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->ctrlops->set_osc_dac(bdev->self, val);
}

int xtrxll_get_osc_freq(struct xtrxll_dev* dev, uint32_t *regval)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->ctrlops->get_osc_freq(bdev->self, regval);
}

/*
int xtrxll_set_txmmcm(struct xtrxll_dev* dev, uint16_t reg, uint16_t value)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->ctrlops->set_txmmcm(bdev->self, reg, value);
}

int xtrxll_get_txmmcm(struct xtrxll_dev* dev, uint16_t* value,
					  uint8_t* locked, uint8_t* rdy)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->ctrlops->get_txmmcm(bdev->self, value, locked, rdy);
}
*/

int xtrxll_repeat_tx_buf(struct xtrxll_dev* dev, int chan, xtrxll_fe_t fmt,
						 const void* buff, unsigned buf_szs, xtrxll_mode_t mode)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->repeat_tx_buf(bdev->self, chan, fmt, buff,
											 buf_szs, mode);
}

int xtrxll_repeat_tx_start(struct xtrxll_dev* dev, int chan, int start)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->selfops->repeat_tx_start(bdev->self, chan, start);
}

int xtrxll_read_uart(struct xtrxll_dev* dev, unsigned uartno,
					 uint8_t* out, unsigned maxsize, unsigned *written)
{
	struct xtrxll_base_dev* bdev = (struct xtrxll_base_dev*)dev;
	return bdev->ctrlops->read_uart(bdev->self, uartno, out, maxsize,
										 written);
}

