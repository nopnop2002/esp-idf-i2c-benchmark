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

static const char *TAG = "i2cdev_legacy";

#define I2CDEV_TIMEOUT 1000

esp_err_t i2c_dev_init(const i2c_dev_t *dev) 
{
	ESP_LOGI(TAG, "Start");
	if (!dev) return ESP_ERR_INVALID_ARG;

	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = dev->sda_io_num,
		.scl_io_num = dev->scl_io_num,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = dev->clk_speed
	};
	ESP_ERROR_CHECK(i2c_param_config(dev->port, &i2c_config));
	ESP_ERROR_CHECK(i2c_driver_install(dev->port, I2C_MODE_MASTER, 0, 0, 0));
	return ESP_OK;
}

#if 0
esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type)
{
	if (!dev) return ESP_ERR_INVALID_ARG;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, dev->addr << 1 | (operation_type == I2C_DEV_READ ? 1 : 0), true);
	i2c_master_stop(cmd);

	esp_err_t res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(I2CDEV_TIMEOUT));

	i2c_cmd_link_delete(cmd);

	return res;
}
#endif

esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data, size_t out_size, void *in_data, size_t in_size)
{
	if (!dev || !in_data || !in_size) return ESP_ERR_INVALID_ARG;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	if (out_data && out_size)
	{
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, dev->addr << 1, true);
		i2c_master_write(cmd, (void *)out_data, out_size, true);
	}
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->addr << 1) | 1, true);
	i2c_master_read(cmd, in_data, in_size, I2C_MASTER_LAST_NACK);
	i2c_master_stop(cmd);

	esp_err_t res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(I2CDEV_TIMEOUT));
	if (res != ESP_OK)
		ESP_LOGE(TAG, "Could not read from device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));

	i2c_cmd_link_delete(cmd);

	return res;
}

esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg, size_t out_reg_size, const void *out_data, size_t out_size)
{
	if (!dev || !out_data || !out_size) return ESP_ERR_INVALID_ARG;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, dev->addr << 1, true);
	if (out_reg && out_reg_size)
		i2c_master_write(cmd, (void *)out_reg, out_reg_size, true);
	i2c_master_write(cmd, (void *)out_data, out_size, true);
	i2c_master_stop(cmd);

	esp_err_t res = i2c_master_cmd_begin(dev->port, cmd, pdMS_TO_TICKS(I2CDEV_TIMEOUT));
	if (res != ESP_OK)
		ESP_LOGE(TAG, "Could not write to device [0x%02x at %d]: %d (%s)", dev->addr, dev->port, res, esp_err_to_name(res));
	i2c_cmd_link_delete(cmd);

	return res;
}
