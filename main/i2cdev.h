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
 * @file i2cdev.h
 * @defgroup i2cdev i2cdev
 * @{
 *
 * ESP-IDF I2C master thread-safe functions for communication with I2C slave
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __I2CDEV_H__
#define __I2CDEV_H__

#include <freertos/FreeRTOS.h>
#include <esp_err.h>
#if CONFIG_LEGACY_DRIVER
#include <driver/i2c.h>
#else
#include <driver/i2c_master.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * I2C device descriptor
 */
typedef struct
{
	int sda_io_num;                     //!< GPIO of SDA
	int scl_io_num;                     //!< GPIO of SCL
	i2c_port_t port;                    //!< I2C port number
	uint8_t addr;                       //!< I2C device address
	uint32_t clk_speed;                 //!< I2C clock frequency
} i2c_dev_t;

/**
 * I2C transaction type
 */
typedef enum {
    I2C_DEV_WRITE = 0, /**< Write operation */
    I2C_DEV_READ       /**< Read operation */
} i2c_dev_type_t;

/**
 * @brief Initialize I2C
 *
 * @param dev Device descriptor
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_init(const i2c_dev_t *dev);

/**
 * @brief Check the availability of the device
 *
 * Issue an operation of \p operation_type to the I2C device then stops.
 *
 * @param dev Device descriptor
 * @param operation_type Operation type
 * @return ESP_OK if device is available
 */
esp_err_t i2c_dev_probe(const i2c_dev_t *dev, i2c_dev_type_t operation_type);

/**
 * @brief Read from slave device
 *
 * Issue a send operation of \p out_data register address, followed by reading \p in_size bytes
 * from slave into \p in_data .
 * Function is thread-safe.
 *
 * @param dev Device descriptor
 * @param out_data Pointer to data to send if non-null
 * @param out_size Size of data to send
 * @param[out] in_data Pointer to input data buffer
 * @param in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read(const i2c_dev_t *dev, const void *out_data,
        size_t out_size, void *in_data, size_t in_size);

/**
 * @brief Write to slave device
 *
 * Write \p out_size bytes from \p out_data to slave into \p out_reg register address.
 * Function is thread-safe.
 *
 * @param dev Device descriptor
 * @param out_reg Pointer to register address to send if non-null
 * @param out_reg_size Size of register address
 * @param out_data Pointer to data to send
 * @param out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write(const i2c_dev_t *dev, const void *out_reg,
        size_t out_reg_size, const void *out_data, size_t out_size);

#if 0
/**
 * @brief Read from register with an 8-bit address
 *
 * Shortcut to ::i2c_dev_read().
 *
 * @param dev Device descriptor
 * @param reg Register address
 * @param[out] in_data Pointer to input data buffer
 * @param in_size Number of byte to read
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_read_reg(const i2c_dev_t *dev, uint8_t reg,
        void *in_data, size_t in_size);

/**
 * @brief Write to register with an 8-bit address
 *
 * Shortcut to ::i2c_dev_write().
 *
 * @param dev Device descriptor
 * @param reg Register address
 * @param out_data Pointer to data to send
 * @param out_size Size of data to send
 * @return ESP_OK on success
 */
esp_err_t i2c_dev_write_reg(const i2c_dev_t *dev, uint8_t reg,
        const void *out_data, size_t out_size);
#endif


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __I2CDEV_H__ */
