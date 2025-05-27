/* i2c - Only read example

   Simple I2C example that shows how to initialize I2C
   as well as reading a sensor connected over I2C.

   The sensor used in this example is a LM75A temperature measurement unit.

   See README.md file to get detailed usage of this example.

*/


#include <stdint.h>
#include <stdio.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/i2c_master.h>
#include "driver/i2c.h"

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           22                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                           /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       1000			/*!< I2C timeout for reading temperature register */

#define I2C_DATA_LENGTH             2				/*!< Size, in bytes, of the read buffer */				

#define LM75A_SENSOR_ADDR           0b1001100                   /*!< Slave address of the LM75A sensor */
#define LM75A_RESOLUTION            0.125			/*!< Resolution of the ADC of the LM75A sensor */


/*
 * float data_to_temperature()
 *
 * This function transforms the buffer data to a absolute float value; however, the temperature data is in 2's complement. This function
 * needs more development to consider that the buffer_data is in 2's complement and, then, maintain the fidelity for temperatures below zero.
 *
 */

float data_to_temperature(uint8_t *data_rd){

    uint16_t temperature_data;
    float temperature;

    temperature_data = ((uint16_t)*data_rd << 3) + ((uint16_t)*(data_rd+sizeof(uint8_t)) >> 5);		// The two bytes of the buffer are appended.
    temperature = LM75A_RESOLUTION*(float)temperature_data;						// The data is converted to a float value.

    return temperature;
    
}

/*
 * void i2c_create_config()
 *
 * This function defines the configuration necessary for the I2C comunication to work.
 *
 */

void i2c_create_config(uint16_t device_address, i2c_master_dev_handle_t *dev_handle, i2c_master_bus_handle_t *bus_handle) {

    i2c_master_bus_config_t i2c_mst_config = {		// Bus configuration.
        .clk_source = I2C_CLK_SRC_DEFAULT,		
        .i2c_port = I2C_MASTER_NUM,			// I2C port. A chip may have more than one I2C ports.
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, bus_handle));			// The bus handle is initalized.

    i2c_device_config_t dev_cfg = {			// Device configuration.
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = I2C_CLK_SRC_DEFAULT,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_cfg, dev_handle));	// The device handle is initialized.
    ESP_LOGI(TAG, "I2C initialized successfully");					// Log initialization message. 
}

void app_main(void){

    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_handle_t bus_handle; 
    i2c_create_config(LM75A_SENSOR_ADDR, &dev_handle, &bus_handle);

    uint8_t * temperature_data = malloc(2*sizeof(uint8_t));

    while(1){
	i2c_master_receive(dev_handle, temperature_data, I2C_DATA_LENGTH, I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS);
	printf("Temperature is: %f \n", data_to_temperature(temperature_data)); 
    	vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    i2c_del_master_bus(bus_handle);
    i2c_master_bus_rm_device(dev_handle);
    
}
