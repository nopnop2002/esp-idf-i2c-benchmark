/* The example of ESP-IDF
 *
 * This sample code is in the public domain.
 */

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h" // ets_get_cpu_frequency()
#include <mcp23017.h>

static const char *TAG = "MAIN";

#define DEBUG 0

void task(void *pvParameter)
{
	if(DEBUG) ESP_LOGW(TAG, "It's running in debug mode");
	i2c_dev_t dev;
#if CONFIG_I2C_PORT_0
	i2c_port_t port = I2C_NUM_0;
#else
	i2c_port_t port = I2C_NUM_1;
#endif
	mcp23015_init(&dev, CONFIG_I2C_SDA_GPIO, CONFIG_I2C_SCL_GPIO, port, CONFIG_I2C_ADDR, CONFIG_I2C_CLOCK_SPEED);
	mcp23017_port_set_mode(&dev, 0x0000);

	TickType_t startTick, endTick, diffTick;
	startTick = xTaskGetTickCount();
	int loop = 10000;
	if(DEBUG) loop=10;
	for (int i=0;i<loop;i++) {
		mcp23017_set_level(&dev, 0, 1);
		if(DEBUG)vTaskDelay(100);
		mcp23017_set_level(&dev, 0, 0);
		if(DEBUG)vTaskDelay(100);
	}
	endTick = xTaskGetTickCount();
	diffTick = endTick - startTick;
#ifdef CONFIG_IDF_TARGET_ESP32
	ESP_LOGI(TAG, "Core is ESP32@%"PRIu32"Mhz", ets_get_cpu_frequency());
#elif defined CONFIG_IDF_TARGET_ESP32S2
	ESP_LOGI(TAG, "Core is ESP32S2@%"PRIu32"Mhz", ets_get_cpu_frequency());
#elif defined CONFIG_IDF_TARGET_ESP32S3
	ESP_LOGI(TAG, "Core is ESP32S3@%"PRIu32"Mhz", ets_get_cpu_frequency());
#elif defined CONFIG_IDF_TARGET_ESP32C2
	ESP_LOGI(TAG, "Core is ESP32C2@%"PRIu32"Mhz", ets_get_cpu_frequency());
#elif defined CONFIG_IDF_TARGET_ESP32C3
	ESP_LOGI(TAG, "Core is ESP32C3@%"PRIu32"Mhz", ets_get_cpu_frequency());
#elif defined CONFIG_IDF_TARGET_ESP32H2
	ESP_LOGI(TAG, "Core is ESP32H2@%"PRIu32"Mhz", ets_get_cpu_frequency());
#endif
	ESP_LOGI(TAG, "loop:%d elapsed time[ms]:%"PRIu32,loop,diffTick*portTICK_PERIOD_MS);
	vTaskDelete( NULL );
}

void app_main()
{
	xTaskCreate(&task, "Task1", 1024*4, NULL, 2, NULL);
}

