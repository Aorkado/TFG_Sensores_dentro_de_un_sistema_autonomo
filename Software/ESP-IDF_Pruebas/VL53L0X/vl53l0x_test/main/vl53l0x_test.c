/* i2c - Only read example

   Simple I2C example that shows how to initialize I2C
   as well as reading a sensor connected over I2C.

   The sensor used in this example is a LM75A temperature measurement unit.

   See README.md file to get detailed usage of this example.

*/

#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "hal/i2c_types.h"
#include "soc/i2c_periph.h"
#include "hal/i2c_hal.h"


static const char *TAG = "i2c-simple-example";
#define ACK_CHECK_EN true

#define I2C_MASTER_SCL_IO           22                          /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                          /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                           /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                      /*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       1000			/*!< I2C timeout for reading temperature register */

#define I2C_DATA_LENGTH             2				/*!< Size, in bytes, of the read buffer */				

#define LM75A_SENSOR_ADDR           0b1001100                   /*!< Slave address of the LM75A sensor */
#define LM75A_RESOLUTION            0.125			/*!< Resolution of the ADC of the LM75A sensor */
#define LM75A_TEMP_INDEX	    0

#define I2C_TRANS_BUF_MINIMUM_SIZE     (sizeof(i2c_cmd_desc_t) + \
                                        sizeof(i2c_cmd_link_t) * 8) /* It is required to have allocate one i2c_cmd_desc_t per command:
                                                                     * start + write (device address) + write buffer +
                                                                     * start + write (device address) + read buffer + read buffer for NACK +
                                                                     * stop */
#if CONFIG_SPIRAM_USE_MALLOC
#define I2C_MEM_ALLOC_CAPS_INTERNAL     (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#endif
#define I2C_MEM_ALLOC_CAPS_DEFAULT      MALLOC_CAP_DEFAULT

typedef struct {
 
    /*!< user specific field */
    uint8_t   i2c_address;                /*!< i2c device address user specific field */
    i2c_port_t i2c_port_num; // ESP32 i2c port number
 
} VL53L0X_Dev_t;

typedef struct {
    i2c_ll_hw_cmd_t hw_cmd;
    union {
        uint8_t* data;      // When total_bytes > 1
        uint8_t data_byte;  //when total_byte == 1
    };
    size_t bytes_used;
    size_t total_bytes;
} i2c_cmd_t;

typedef struct i2c_cmd_link {
    i2c_cmd_t cmd;              /*!< command in current cmd link */
    struct i2c_cmd_link *next;  /*!< next cmd link */
} i2c_cmd_link_t;

typedef struct {
    i2c_cmd_link_t *head;     /*!< head of the command link */
    i2c_cmd_link_t *cur;      /*!< last node of the command link */
    i2c_cmd_link_t *free;     /*!< the first node to free of the command link */

    void     *free_buffer;    /*!< pointer to the next free data in user's buffer */
    uint32_t  free_size;      /*!< remaining size of the user's buffer */
} i2c_cmd_desc_t;

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

esp_err_t i2c_init() {

    i2c_port_t i2c_master_port = 0;
    i2c_config_t conf={0};
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_MASTER_SDA_IO;
	conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
	conf.scl_io_num = I2C_MASTER_SCL_IO;
	conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);

}

esp_err_t VL53L0X_ReadMulti(VL53L0X_Dev_t* Dev, uint8_t index, uint8_t *pdata, uint32_t size)
{
    esp_err_t err = ESP_OK;
    uint8_t buffer[I2C_TRANS_BUF_MINIMUM_SIZE] = { 0 };

    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
    assert(handle != NULL);

    err = i2c_master_start(handle);
    if (err != ESP_OK) {
        goto end;
    }
    
    err = i2c_master_write_byte(handle, Dev->i2c_address << 1 | I2C_MASTER_READ, true);
    if (err != ESP_OK) {
        goto end;
    }

    err = i2c_master_read(handle, pdata, size, I2C_MASTER_LAST_NACK);
    if (err != ESP_OK) {
        goto end;
    }

    i2c_master_stop(handle);
    err = i2c_master_cmd_begin(Dev->i2c_port_num, handle, 1000/portTICK_PERIOD_MS);

end:
    i2c_cmd_link_delete_static(handle);
    return err;

}

void app_main(void){

    VL53L0X_Dev_t dev={0};
    	dev.i2c_address = LM75A_SENSOR_ADDR;
    	dev.i2c_port_num = 0;

    ESP_ERROR_CHECK(i2c_init());
    ESP_LOGI(TAG, "I2C initialized successfully");					// Log initialization message. 
    
    uint8_t * temperature_data = malloc(2*sizeof(uint8_t));

    while(1){
	
	ESP_ERROR_CHECK(VL53L0X_ReadMulti(&dev, LM75A_TEMP_INDEX, temperature_data, I2C_DATA_LENGTH));
	//i2c_master_read_from_device(dev.i2c_port_num, dev.i2c_address, temperature_data, 2, 1000/portTICK_PERIOD_MS);
	printf("Temperature is: %f \n", data_to_temperature(temperature_data)); 
    	vTaskDelay(100 / portTICK_PERIOD_MS);
    }

}
