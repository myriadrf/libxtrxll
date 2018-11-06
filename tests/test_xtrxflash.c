/*
 * xtrx flash utility file
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
#include "xtrxll_flash.h"
#include "xtrxll_log.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

int main(int argc, char** argv)
{
	struct xtrxll_dev *odev;
	int opt;
	const char* device = NULL;
	const char* read_filename = NULL;
	unsigned read_flash_off = 0;
	unsigned read_file_size = 2048*1024;
	const char* write_filename = NULL;
	unsigned write_flash_off = 0;
	unsigned write_file_size = 0;

	int dont_write_verify = 0;
	int show_info = 0;
	int loglevel = 2;
	//int erase_bulk = 0;
	int memdump = 0;
	int bitformat = 0;
	int res;
	struct xtrxll_base_dev *dev;
	unsigned vio_program = 2700;
	unsigned vio_read = 2300;

	while ((opt = getopt(argc, argv, "d:o:r:s:l:iO:S:w:neDIV:v:")) != -1) {
		switch (opt) {
		case 'v':
			vio_read = atoi(optarg);
			break;
		case 'V':
			vio_program = atoi(optarg);
			break;
		case 'd':
			device = optarg;
			break;
		case 'l':
			loglevel = atoi(optarg);
			break;
		case 'r':
			read_filename = optarg;
			break;
		case 'o':
			read_flash_off = atoi(optarg);
			break;
		case 's':
			read_file_size = atoi(optarg);
			break;
		case 'w':
			write_filename = optarg;
			break;
		case 'O':
			write_flash_off = atoi(optarg);
			break;
		case 'S':
			write_file_size = atoi(optarg);
			break;
		case 'i':
			show_info = 1;
			break;
		case 'n':
			dont_write_verify = 1;
			break;
		case 'e':
			//erase_bulk = 1;
			break;
		case 'D':
			memdump = 1;
			break;
		case 'I':
			bitformat = 1;
			break;
		default: /* '?' */
			fprintf(stderr, "Usage: %s [-d device] [-r out_filename] [-o read_flash_off] [-s read_file_size] [-w in_filename] [-O write_flash_off] [-S write_file_size] [-i] [-n] [-l loglevel]\n",
					argv[0]);
			exit(EXIT_FAILURE);
		}

	}

	xtrxll_set_loglevel(loglevel);

	res = xtrxll_open(device, 0, &odev);
	if (res)
		goto falied_open;

	dev = (struct xtrxll_base_dev *)odev;

	if (memdump) {
		uint32_t mem[64];
		res = dev->ctrlops->mem_rb32(dev->self, 0, 64, &mem[0]);
		if (res < 0)
			goto falied_upl;

		for (int i = 0; i < 64; i++)
			printf("%d:\t%08x\n", i, mem[i]);
	}

	res = xtrxll_set_param(odev, XTRXLL_PARAM_PWR_CTRL, PWR_CTRL_BUSONLY);
	if (res) {
		fprintf(stderr, "Unable to reset LMS power");
		goto falied_upl;
	}

	usleep(50000);

	if (vio_read > 1000) {
		res = xtrxll_set_param(odev, XTRXLL_PARAM_PWR_VIO, vio_read);
		if (res) {
			fprintf(stderr, "Unable to set VIO to %dmV for reading\n", vio_read);
			goto falied_upl;
		}
		usleep(10000);
	}

	if (!read_filename && !write_filename)
		show_info = 1;

	if (show_info) {
		uint32_t flash_id;
		uint32_t capacity;
		char str[512];
		res = xtrxll_flash_get_id(dev, &flash_id, &capacity, str, 511);
		if (res)
			goto falied_upl;

		printf("Flash ID: %08x, capacity %d bytes (%s)\n",
			   flash_id, capacity, str);
	}

	if (read_filename) {
		char *mem = (char*)malloc(read_file_size + 4);
		if (mem == NULL)
			goto falied_upl;

		FILE* f = fopen(read_filename, "w+b");
		if (f) {
			res = xtrxll_flash_to_host(dev, read_flash_off,
									   read_file_size, mem);
			if (!res) {
				fwrite(mem, 1, read_file_size, f);
			}
			fclose(f);
		} else {
			res = errno;
			fprintf(stderr, "Unable to create file `%s`", strerror(res));
		}
		free(mem);

		if (res)
			goto falied_upl;

		fprintf(stderr, "Data has been read successfully (%d bytes)\n", read_file_size);
	}

	if (write_filename) {
		res = xtrxll_set_param(odev, XTRXLL_PARAM_PWR_VIO, vio_program);
		if (res) {
			fprintf(stderr, "Unable to set VIO to %dmV for programming\n", vio_program);
			goto falied_upl;
		}
		usleep(1000);

		const unsigned block_size = 4096;
		char *mem = NULL;
		uint32_t rounded_size;
		FILE* f = fopen(write_filename, "r+b");
		if (f) {
			if (write_file_size == 0) {
				fseek(f, 0, SEEK_END);
				write_file_size = (unsigned)ftell(f);

				if (bitformat) {
					fseek(f, 120, SEEK_SET); // FIXME hardcoded offset for BIT format
					write_file_size -= 120;
				} else {
					fseek(f, 0, SEEK_SET);
				}

				fprintf(stderr, "File size is %d\n", write_file_size);
			}

			rounded_size = ((write_file_size + block_size - 1) / block_size) * block_size;
			mem = (char*)malloc(rounded_size);
			if (mem == NULL) {
				fclose(f);
				goto falied_upl;
			}
			if ((rounded_size - write_file_size)) {
				memset(mem + write_file_size, -1, (rounded_size - write_file_size));
				fprintf(stderr, "Padding %u bytes with 0xff to meet sector size\n",
						rounded_size - write_file_size);
			}

			size_t read = fread(mem, 1, write_file_size, f);
			fclose(f);

			if (read != write_file_size) {
				fprintf(stderr, "Unable to read %d bytes!\n", write_file_size);
				free(mem);
				goto falied_upl;
			}
		} else {
			res = errno;
			fprintf(stderr, "Unable to open file `%s`", strerror(res));
			goto falied_upl;
		}

		res = xtrxll_flash_from_host(dev, mem,
									 rounded_size, write_flash_off, 0);

		if (res) {
			fprintf(stderr, "Error during programming: %d\n", res);
			free(mem);
			goto falied_upl;
		}

		if (!dont_write_verify) {
			fprintf(stderr, "Verifying...\n");

			char* mem2 = (char*)malloc(rounded_size);
			if (mem2 == NULL) {
				free(mem);
				goto falied_upl;
			}

			res = xtrxll_flash_to_host(dev, write_flash_off,
									   rounded_size, mem2);
			if (res) {
				free(mem2);
				free(mem);
				goto falied_upl;
			}

			unsigned i;
			for (i = 0; i < rounded_size; i++) {
				if (mem[i] != mem2[i]) {
					fprintf(stderr, "Program error at %d offset (%d) write %x read %x\n",
							write_flash_off + i, i,
							((unsigned char*)mem)[i],
							((unsigned char*)mem2)[i]);
					break;
				}
			}

			if (i == rounded_size) {
				fprintf(stderr, "Write successful %u+%u bytes!\n",
						write_file_size, rounded_size - write_file_size);
			}
			free(mem2);
		}
		free(mem);
	}

	res = 0;

falied_upl:
	xtrxll_set_param(odev, XTRXLL_PARAM_PWR_CTRL, PWR_CTRL_PDOWN);
	xtrxll_close(odev);
falied_open:
	return res;
}
