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
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include <inttypes.h>
#include "driver/gpio.h"


static const char *TAG = "i2c-simple-example";
#define ACK_CHECK_EN true

#define I2C_MASTER_SCL_IO           	22			/*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           	21			/*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              	0			/*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          	400000			/*!< I2C master clock frequency */
#define I2C_MASTER_TIMEOUT_MS       	1000			/*!< I2C timeout for reading temperature register */

#define I2C_DATA_LENGTH             	2			/*!< Size, in bytes, of the read buffer */				

#define VL53L0X_DEFAULT_ADDR           	0x29			/*!< Default address of the VL53L0X sensor */

#define GPIO_OUTPUT_L		  	25			/*!< Number of the GPIOS that are going to be used */
#define GPIO_OUTPUT_C		  	26
#define GPIO_OUTPUT_PIN_SEL		(1ULL<<GPIO_OUTPUT_L)|(1ULL<<GPIO_OUTPUT_C)



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
                        &VhvSettings, &PhaseCal); // Device Calibration
        print_pal_error(Status);
 
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
                        &refSpadCount, &isApertureSpads); // Device Spad Management
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
        for(i=0;i<2;i++){
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

void device_info(VL53L0X_Dev_t *pDevice){

    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_DeviceInfo_t DeviceInfo; 

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = VL53L0X_GetDeviceInfo(pDevice, &DeviceInfo);
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
}

VL53L0X_Error VL53L0X_BootAndSetDevAddr(gpio_num_t gpio_num, VL53L0X_Dev_t *pDevice, uint8_t i2c_dev_addr){

    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	
    // Boot device
    gpio_set_level(gpio_num, 1);   
    vTaskDelay(150 / portTICK_PERIOD_MS);

    // Set new direction of device
    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_SetDeviceAddress\n");
        Status = VL53L0X_SetDeviceAddress(pDevice,(i2c_dev_addr << 1)); 	// The i2c address of 7 bits has to be shifted one bit to the left for SetDeviceAddress to funtion properly. 
        print_pal_error(Status);
    }

    pDevice->i2c_address = i2c_dev_addr;
      
    return Status;
}

void app_main(void){

    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    VL53L0X_Dev_t device_1;
    VL53L0X_Dev_t *pdevice_1 = &device_1;
    VL53L0X_Dev_t device_2;
    VL53L0X_Dev_t *pdevice_2 = &device_2;
 
    //VL53L0X_Version_t Version;
    //VL53L0X_Version_t *pVersion   = &Version;
    //VL53L0X_DeviceInfo_t first_device_info, second_device_info;
 
    uint8_t i2c_dev_addr_1 = 0x10;
    uint8_t i2c_dev_addr_2 = 0x11;


    // Configuration of the I2C master and bus
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

    // Configuration of the I2C parameters of the VL53L0X devices
    pdevice_1->i2c_address = 0x29;
    pdevice_1->i2c_port_num = I2C_NUM_1;
    pdevice_1->comms_type = 1;
    pdevice_1->comms_speed_khz = 400;
    
    pdevice_2->i2c_address = 0x29;
    pdevice_2->i2c_port_num = I2C_NUM_1;
    pdevice_2->comms_type = 1;
    pdevice_2->comms_speed_khz = 400;
 
    // Configuration of the GPIO peripheral
    gpio_initialization();

    // Boot and configuration of the VL53L0X device
    printf("Booting and setting new device address for the two vl53l0x.\n");
    if(Status == VL53L0X_ERROR_NONE)
    {
    	Status = VL53L0X_BootAndSetDevAddr(GPIO_OUTPUT_L,pdevice_1,i2c_dev_addr_1);
    	Status = VL53L0X_BootAndSetDevAddr(GPIO_OUTPUT_C,pdevice_2,i2c_dev_addr_2);
	print_pal_error(Status);
    }
 
    // Get direction of devices
    printf("Direction of first device: %d \n",pdevice_1->i2c_address);
    printf("Direction of second device: %d \n",pdevice_2->i2c_address);
 
    //
    if(Status == VL53L0X_ERROR_NONE)
    {
	Status = VL53L0X_DataInit(pdevice_1);
    	Status = VL53L0X_DataInit(pdevice_2);
	print_pal_error(Status);
    } 

    printf("Getting info of first device.\n");
    device_info(pdevice_1);
    printf("Getting info of second device.\n");
    device_info(pdevice_2);


    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = rangingTest(pdevice_2);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	Status = rangingTest(pdevice_1);
 
    }
    

}
