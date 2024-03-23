/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file i2cdev.c
 *
 * ESP-IDF I2C master thread-safe functions for communication with I2C slave
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#include <string.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include "i2cdev.h"

static const char *TAG = "i2cdev_new";

#define I2CDEV_TIMEOUT 1000

i2c_master_dev_handle_t dev_handle;

esp_err_t i2c_dev_init(const i2c_dev_t *dev) 
{
	ESP_LOGI(TAG, "Start");
	if (!dev) return ESP_ERR_INVALID_ARG;

	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.i2c_port = dev->port,
		.scl_io_num = dev->scl_io_num,
		.sda_io_num = dev->sda_io_num,
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = dev->addr,
		.scl_speed_hz = dev->clk_speed,
	};
	//i2c_master_dev_handle_t dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

	return ESP_OK;
}

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
	if (!dev || !in_data || !in_size) return ESP_ERR_INVALID_ARG;

	esp_err_t res;
	if (out_data && out_size)
	{
		res = i2c_master_transmit_receive(dev_handle, out_data, out_size, in_data, in_size, pdMS_TO_TICKS(I2CDEV_TIMEOUT));
	} else {
		res = i2c_master_receive(dev_handle, in_data, in_size, pdMS_TO_TICKS(I2CDEV_TIMEOUT));
	}
	if (res != ESP_OK)
		ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));

	return res;
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
	if (!dev || !out_data || !out_size) return ESP_ERR_INVALID_ARG;

	void*  out_buf;	
	out_buf = malloc(out_reg_size + out_size);
	if (out_buf == NULL) {
		ESP_LOGE(TAG, "malloc fail");
		return ESP_ERR_NO_MEM;
	}	
	memcpy(out_buf, out_reg, out_reg_size);
	memcpy(out_buf+out_reg_size, out_data, out_size);

	esp_err_t res;
	res = i2c_master_transmit(dev_handle, out_buf, out_reg_size + out_size, pdMS_TO_TICKS(I2CDEV_TIMEOUT));
	if (res != ESP_OK)
		ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
	free(out_buf);

	return res;
}
