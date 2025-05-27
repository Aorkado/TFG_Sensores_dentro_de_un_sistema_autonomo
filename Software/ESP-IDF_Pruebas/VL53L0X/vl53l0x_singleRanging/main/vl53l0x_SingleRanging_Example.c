#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "required_version.h"
#include "get_nucleo_serial_comm.h"
#include <malloc.h>


esp_err_t vl53l0x_to_esp_error(VL53L0X_Error error) {

    switch (VL53L0X_Error) {
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


VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    esp_err_t ret = vl53l0x_to_esp_error(Status);
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

void app_main()
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice = &MyDevice;
    VL53L0X_Version_t                   Version;
    VL53L0X_Version_t                  *pVersion   = &Version;
    VL53L0X_DeviceInfo_t                DeviceInfo;

    int32_t status_int;
    int32_t init_done = 0;
    TCHAR SerialCommStr[MAX_VALUE_NAME];

    // Initialize Comms
    pMyDevice->I2cDevAddr      = 0x52;
    pMyDevice->comms_type      =  1;
    pMyDevice->comms_speed_khz =  400;

    Status = VL53L0X_i2c_init(SerialCommStr, 460800);

    ESP_ERROR_CHECK(vl53l0x_to_esp_error(Status));
    if (Status != VL53L0X_ERROR_NONE) {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
        init_done = 1;
    } else
        printf ("Init Comms\n");

    /*
     * Disable VL53L0X API logging if you want to run at full speed
     */
#ifdef VL53L0X_LOG_ENABLE
    VL53L0X_trace_config("test.log", TRACE_MODULE_ALL, TRACE_LEVEL_ALL, TRACE_FUNCTION_ALL);
#endif

    /*
     *  Get the version of the VL53L0X API running in the firmware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        status_int = VL53L0X_GetVersion(pVersion);
        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    ESP_ERROR_CHECK(vl53l0x_to_esp_error(Status));
    /*
     *  Verify the version of the VL53L0X API running in the firmware
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        if( pVersion->major != VERSION_REQUIRED_MAJOR ||
            pVersion->minor != VERSION_REQUIRED_MINOR ||
            pVersion->build != VERSION_REQUIRED_BUILD )
        {
            printf("VL53L0X API Version Error: Your firmware has %d.%d.%d (revision %d). This example requires %d.%d.%d.\n",
                pVersion->major, pVersion->minor, pVersion->build, pVersion->revision,
                VERSION_REQUIRED_MAJOR, VERSION_REQUIRED_MINOR, VERSION_REQUIRED_BUILD);
        }
    }


    if(Status == VL53L0X_ERROR_NONE)
    {
        printf ("Call of VL53L0X_DataInit\n");
        Status = VL53L0X_DataInit(&MyDevice); // Data initialization
        print_pal_error(Status);
	ESP_ERROR_CHECK(vl53l0x_to_esp_error(Status));	 
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
	ESP_ERROR_CHECK(vl53l0x_to_esp_error(Status));
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        Status = rangingTest(pMyDevice);
    }

    print_pal_error(Status);
    ESP_ERROR_CHECK(vl53l0x_to_esp_error(Status));
    // Implementation specific

    /*
     *  Disconnect comms - part of VL53L0X_platform.c
     */

    if(init_done == 0)
    {
        printf ("Close Comms\n");
        status_int = VL53L0X_comms_close();
        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    print_pal_error(Status);
    ESP_ERROR_CHECK(vl53l0x_to_esp_error(Status));
//    printf ("\nPress a Key to continue!");
//    getchar();
    
}

