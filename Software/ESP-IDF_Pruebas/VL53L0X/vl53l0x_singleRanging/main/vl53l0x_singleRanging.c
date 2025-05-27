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
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include <inttypes.h>
#include "driver/gpio.h"
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

#define GPIO_OUTPUT_C		    26 
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_C)


esp_err_t vl53l0x_to_esp_error(VL53L0X_Error error) {
         
    switch (error) {
        case VL53L0X_ERROR_NONE:
            return ESP_OK;
        case VL53L0X_ERROR_INVALID_PARAMS:
            return ESP_ERR_INVALID_ARG;
        case VL53L0X_ERROR_CONTROL_INTERFACE:
            return ESP_ERR_INVALID_STATE;
        case VL53L0X_ERROR_TIME_OUT:
            return ESP_ERR_TIMEOUT;
        default:
            return ESP_FAIL;
	}
}

void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %i : %s\n", Status, buf);
}

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;
    
    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */
        
    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    printf("Range Status: %i : %s\n", RangeStatus, buf);

}

void gpio_initialization(){

    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

}
VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    int i;
    FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
        print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice,
                        &VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(Status);
 
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
                        &refSpadCount, &isApertureSpads); // Device Initialization
        printf ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
        print_pal_error(Status);

    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        printf ("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
 
    }

    // Enable/Disable Sigma and Signal check
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
                        VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
                        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
                        VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
                        VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                        (FixPoint1616_t)(1.5*0.023*65536));
    }


    /*
     *  Step  4 : Test ranging mode
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        for(i=0;i<10;i++){
            printf ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
                        &RangingMeasurementData);

            print_pal_error(Status);
 
            print_range_status(&RangingMeasurementData);

            VL53L0X_GetLimitCheckCurrent(pMyDevice,
                        VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

            printf("RANGE IGNORE THRESHOLD: %f\n\n", (float)LimitCheckCurrent/65536.0);


            if (Status != VL53L0X_ERROR_NONE) break;

            printf("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);


        }
    }
    return Status;
}



void app_main(void){

    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice = &MyDevice;
    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    VL53L0X_DeviceInfo_t                DeviceInfo;

    i2c_port_t i2c_master_port = I2C_NUM_1;
    i2c_config_t conf={0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = 22;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;

    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0));

    pMyDevice->i2c_address = 0x29;
    pMyDevice->i2c_port_num = I2C_NUM_1;
    pMyDevice->comms_type = 1;
    pMyDevice->comms_speed_khz = 400;
    
    gpio_initialization();
    gpio_set_level(GPIO_OUTPUT_C, 1);   
    vTaskDelay(250 / portTICK_PERIOD_MS);
    /*
     *  Get the version of the VL53L0X API running in the firmware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        if (VL53L0X_GetVersion(pVersion) != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
	print_pal_error(Status);
    }
    printf("VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d).\n",
                pVersion->major, pVersion->minor, pVersion->build, pVersion->revision);

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_DataInit\n");
        Status = VL53L0X_DataInit(&MyDevice); // Data initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
        if(Status == VL53L0X_ERROR_NONE)
        {
            printf("VL53L0X_GetDeviceInfo:\n");
            printf("Device Name : %s\n", DeviceInfo.Name);
            printf("Device Type : %s\n", DeviceInfo.Type);
            printf("Device ID : %s\n", DeviceInfo.ProductId);
            printf("ProductRevisionMajor : %d\n", DeviceInfo.ProductRevisionMajor);
        printf("ProductRevisionMinor : %d\n", DeviceInfo.ProductRevisionMinor);

        if ((DeviceInfo.ProductRevisionMinor != 1) && (DeviceInfo.ProductRevisionMinor != 1)) {
                printf("Error expected cut 1.1 but found cut %d.%d\n",
                       DeviceInfo.ProductRevisionMajor, DeviceInfo.ProductRevisionMinor);
                Status = VL53L0X_ERROR_NOT_SUPPORTED;
            }
        }
        print_pal_error(Status);
    }
    while(1){

    	if(Status == VL53L0X_ERROR_NONE)
    	{
        	Status = rangingTest(pMyDevice);
    	}
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    print_pal_error(Status);

}
