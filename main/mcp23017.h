/*
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of itscontributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file mcp23017.h
 * @defgroup mcp23017 mcp23017
 * @{
 *
 * ESP-IDF driver for I2C/SPI 16 bit GPIO expanders MCP23017/MCP23S17
 *
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MCP23017_H__
#define __MCP23017_H__

#include <stddef.h>
#include <stdbool.h>
#include <i2cdev.h>
#include <driver/gpio.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * GPIO mode
 */
typedef enum
{
    MCP23017_GPIO_OUTPUT = 0,
    MCP23017_GPIO_INPUT
} mcp23017_gpio_mode_t;

/**
 * INTA/INTB pins mode
 */
typedef enum
{
    MCP23017_ACTIVE_LOW = 0, //!< Low level on interrupt
    MCP23017_ACTIVE_HIGH,    //!< High level on interrupt
    MCP23017_OPEN_DRAIN      //!< Open drain
} mcp23017_int_out_mode_t;

/**
 * Interrupt mode
 */
typedef enum
{
    MCP23017_INT_DISABLED = 0, //!< No interrupt
    MCP23017_INT_LOW_EDGE,     //!< Interrupt on low edge
    MCP23017_INT_HIGH_EDGE,    //!< Interrupt on high edge
    MCP23017_INT_ANY_EDGE      //!< Interrupt on any edge
} mcp23017_gpio_intr_t;

/**
 * @brief Get INTA/INTB pins mode
 *
 * @param dev Pointer to device descriptor
 * @param sda_io_num GPIO of SDA
 * @param scl_io_num GPIO of SCL
 * @param port I2C port number
 * @param addr I2C device address
 * @param clk_speed I2C clock speed[Hz]
 * @return `ESP_OK` on success
 */
esp_err_t mcp23015_init(i2c_dev_t *dev, int sda_io_num, int scl_io_num, i2c_port_t port, uint8_t addr, uint32_t clk_speed);

/**
 * @brief Get INTA/INTB pins mode
 *
 * @param dev Pointer to device descriptor
 * @param[out] mode Buffer to store mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_get_int_out_mode(i2c_dev_t *dev, mcp23017_int_out_mode_t *mode);

/**
 * @brief Set INTA/INTB pins mode
 *
 * @param dev Pointer to device descriptor
 * @param mode INTA/INTB pins mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_set_int_out_mode(i2c_dev_t *dev, mcp23017_int_out_mode_t mode);

/**
 * @brief Get GPIO pins mode
 *
 * 0 - output, 1 - input for each bit in `val`
 *
 * @param dev Pointer to device descriptor
 * @param[out] val Buffer to store mode, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
 * @return
 */
esp_err_t mcp23017_port_get_mode(i2c_dev_t *dev, uint16_t *val);

/**
 * @brief Set GPIO pins mode
 *
 * 0 - output, 1 - input for each bit in `val`
 *
 * @param dev Pointer to device descriptor
 * @param val Mode, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_port_set_mode(i2c_dev_t *dev, uint16_t val);

/**
 * @brief Get GPIO pullups status
 *
 * 0 - pullup disabled, 1 - pullup enabled for each bit in `val`
 *
 * @param dev Pointer to device descriptor
 * @param[out] val Pullup status, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_port_get_pullup(i2c_dev_t *dev, uint16_t *val);

/**
 * @brief Set GPIO pullups status
 *
 * 0 - pullup disabled, 1 - pullup enabled for each bit in `val`
 *
 * @param dev Pointer to device descriptor
 * @param val Pullup status, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_port_set_pullup(i2c_dev_t *dev, uint16_t val);

/**
 * @brief Read GPIO port value
 *
 * @param dev Pointer to device descriptor
 * @param[out] val 16-bit GPIO port value, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_port_read(i2c_dev_t *dev, uint16_t *val);

/**
 * @brief Write value to GPIO port
 *
 * @param dev Pointer to device descriptor
 * @param val GPIO port value, 0 bit for PORTA/GPIO0..15 bit for PORTB/GPIO7
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_port_write(i2c_dev_t *dev, uint16_t val);

/**
 * @brief Get GPIO pin mode
 *
 * @param dev Pointer to device descriptor
 * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
 * @param[out] mode GPIO pin mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_get_mode(i2c_dev_t *dev, uint8_t pin, mcp23017_gpio_mode_t *mode);

/**
 * @brief Set GPIO pin mode
 *
 * @param dev Pointer to device descriptor
 * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
 * @param mode GPIO pin mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_set_mode(i2c_dev_t *dev, uint8_t pin, mcp23017_gpio_mode_t mode);

/**
 * @brief Get pullup mode of GPIO pin
 *
 * @param dev Pointer to device descriptor
 * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
 * @param[out] enable pullup mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_get_pullup(i2c_dev_t *dev, uint8_t pin, bool *enable);

/**
 * @brief Set pullup mode of GPIO pin
 *
 * @param dev Pointer to device descriptor
 * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
 * @param enable `true` to enable pullup
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_set_pullup(i2c_dev_t *dev, uint8_t pin, bool enable);

/**
 * @brief Read GPIO pin level
 *
 * @param dev Pointer to device descriptor
 * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
 * @param[out] val `true` if pin currently in high state
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_get_level(i2c_dev_t *dev, uint8_t pin, uint32_t *val);

/**
 * @brief Set GPIO pin level
 *
 * Pin must be set up as output
 *
 * @param dev Pointer to device descriptor
 * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
 * @param[out] val `true` if pin currently in high state
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_set_level(i2c_dev_t *dev, uint8_t pin, uint32_t val);

/**
 * @brief Setup interrupt for group of GPIO pins
 *
 * @param dev Pointer to device descriptor
 * @param mask Pins to setup
 * @param intr Interrupt mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_port_set_interrupt(i2c_dev_t *dev, uint16_t mask, mcp23017_gpio_intr_t intr);

/**
 * @brief Setup interrupt for GPIO pin
 *
 * @param dev Pointer to device descriptor
 * @param pin Pin number, 0 for PORTA/GPIO0..15 for PORTB/GPIO7
 * @param intr Interrupt mode
 * @return `ESP_OK` on success
 */
esp_err_t mcp23017_set_interrupt(i2c_dev_t *dev, uint8_t pin, mcp23017_gpio_intr_t intr);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MCP23017_H__ */
