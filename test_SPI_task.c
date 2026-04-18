// #include <ti/drivers/SPI.h>
// #include <unistd.h>           // For TI standard POSIX usleep
// #include "ti_drivers_config.h"
// #include "inv_imu_driver.h"

// #include <FreeRTOS.h>
// #include <task.h>
// #include <string.h>
// #include <app_main.h>

// #include "icm_func_impl.h" // contains spi_read_reg, spi_write_reg, and delay_us


// void init_icm45605(void) {
//     int rc = 0;
//     uint8_t whoami;
//     inv_imu_device_t imu_dev; // Create local/global instance variable
    
//     // 1. Initialize and open the TI SPI Driver
//     SPI_Params spiParams;
//     SPI_Params_init(&spiParams);
//     spiParams.bitRate = 1000000;   // Start with 1 MHz; can increase after successful bring-up
//     spiParams.dataSize = 8;
//     spiParams.mode = SPI_MASTER;

//     spiHandle = SPI_open(CONFIG_SPI_0, &spiParams); // Using your SysConfig defined SPI
//     if (spiHandle == NULL) {
//         // Initialization Failed - SPI in use or incorrectly configured
//         while (1);
//     }

//     // 2. Map Transport layer to your implemented functions
//     imu_dev.transport.read_reg   = spi_read_reg;     //
//     imu_dev.transport.write_reg  = spi_write_reg;    //
//     imu_dev.transport.sleep_us   = delay_us;         //
//     imu_dev.transport.serif_type = UI_SPI4;          // 4-wire SPI as configured in your pins

//     // 3. Wait 3ms for the device to properly power on
//     delay_us(3000);                                  //

//     // 4. Verify communication by reading WHO_AM_I register
//     rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);   //
//     if (rc != 0 || whoami != INV_IMU_WHOAMI) {
//         // Handle Error: SPI communication failed or incorrect WHO_AM_I ID returned
//         return; 
//     }

//     // 5. Trigger a soft-reset to put the part into a known state
//     rc |= inv_imu_soft_reset(&imu_dev);              //
    
//     // If you reach here with rc == 0, initialization was successful!
// }



// // Task Configuration 
// #define TASK_STACK_SIZE 1024 
// #define TASK_PRIORITY   1   
// static TaskHandle_t SPI_task_handle = NULL;


// // Task
// static void SPI_task(void *pvParameters)
// {
//     for (;;)
//     {
        
//         vTaskDelay(pdMS_TO_TICKS(1000)); 
//     }
// }

// // Task Creation Function
// void create_SPI_task(void)
// {   
//     xTaskCreate(SPI_task,       
//                 "Test SPI",             
//                 TASK_STACK_SIZE,   
//                 NULL,                     
//                 TASK_PRIORITY,     
//                 &test_task_handle);  
// }


#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h> // Added for serial console output
#include <unistd.h>             // For TI standard POSIX usleep
#include "ti_drivers_config.h"
#include "imu/inv_imu_driver.h" // Fixed path to match your folder structure

#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include <app_main.h> // Ensure you have this in your actual project if needed

#include "icm_func_impl.h" // contains spi_read_reg, spi_write_reg, and delay_us

// Global handles

static inv_imu_device_t imu_dev; // Moved to file scope for task access
static Display_Handle displayHandle;

// void test_spi_loopback(void) {
//     SPI_Transaction spiTransaction = {0};
//     uint8_t txBuffer[4] = {0xDE, 0xAD, 0xBE, 0xEF}; // Dummy test pattern
//     uint8_t rxBuffer[4] = {0};

//     spiTransaction.count = 4;
//     spiTransaction.txBuf = (void *)txBuffer;
//     spiTransaction.rxBuf = (void *)rxBuffer;

//     if (SPI_transfer(spiHandle, &spiTransaction)) {
//         Display_printf(displayHandle, 0, 0, "Loopback Read: %02X %02X %02X %02X", 
//                        rxBuffer[0], rxBuffer[1], rxBuffer[2], rxBuffer[3]);
        
//         if (rxBuffer[0] == 0xDE && rxBuffer[1] == 0xAD) {
//              Display_printf(displayHandle, 0, 0, "SUCCESS: CC2340R5 SPI is working perfectly!");
//         } else {
//              Display_printf(displayHandle, 0, 0, "FAIL: Microcontroller is reading zeroes.");
//         }
//     } else {
//         Display_printf(displayHandle, 0, 0, "FAIL: SPI_transfer crashed.");
//     }
    
//     // Halt here so you can read the console
//     while(1); 
// }

void init_icm45605(void) {
    int rc = 0;
    uint8_t whoami;

    // GPIO_setConfig(6, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH | GPIO_CFG_OUT_STR_MED);
    // GPIO_write(6, 1);

    // 1. Initialize and open the TI SPI Driver
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 1000000;   // Start with 1 MHz; can increase after successful bring-up
    spiParams.dataSize = 8;
    spiParams.mode = SPI_CONTROLLER;


    // Using your SysConfig defined SPI
    spiHandle = SPI_open(CONFIG_SPI_0, &spiParams); 
    // test_spi_loopback();

    if (spiHandle == NULL) {
        Display_printf(displayHandle, 0, 0, "SPI Initialization Failed!");
        spiHandle = SPI_open(CONFIG_SPI_0, &spiParams); 
    }

    // 2. Map Transport layer to your implemented functions
    imu_dev.transport.read_reg   = spi_read_reg;     
    imu_dev.transport.write_reg  = spi_write_reg;    
    imu_dev.transport.sleep_us   = delay_us;         
    imu_dev.transport.serif_type = UI_SPI4;          

    // 3. Wait 3ms for the device to properly power on
    delay_us(3000);                                  

   // 4. Verify communication by reading WHO_AM_I register
    rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);   
    if (rc != 0 || whoami != INV_IMU_WHOAMI) {
        Display_printf(displayHandle, 0, 0, "IMU WHO_AM_I Check Failed! Expected: 0x%x, Read: 0x%x", INV_IMU_WHOAMI, whoami);
        while(1);
        return; 
    } else {
        Display_printf(displayHandle, 0, 0, "IMU WHO_AM_I Check Sucess! Expected: 0x%x, Read: 0x%x", INV_IMU_WHOAMI, whoami);
    }

    // 5. Trigger a soft-reset to put the part into a known state
    rc |= inv_imu_soft_reset(&imu_dev);              

    // 6. Turn on Accelerometer and Gyroscope in Low Noise (LN) mode
    rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
    rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);

    if (rc == 0) {
        Display_printf(displayHandle, 0, 0, "ICM45605 Initialized Successfully!");
    } else {
        Display_printf(displayHandle, 0, 0, "ICM45605 Initialization sequence returned error: %d", rc);
    }
}


// Task Configuration 
#define TASK_STACK_SIZE 2048 
#define TASK_PRIORITY   1   
static TaskHandle_t SPI_task_handle = NULL;

// Task
static void SPI_task(void *pvParameters)
{
    inv_imu_sensor_data_t sensor_data;
    
    // Initialize UART Display for logging
    Display_init();
    displayHandle = Display_open(Display_Type_UART, NULL);
    
    SPI_init();
    
    // Initialize the IMU
    init_icm45605();

    for (;;)
    {
        // Read the 3-axis accel, 3-axis gyro, and temperature
        int rc = inv_imu_get_register_data(&imu_dev, &sensor_data);
        
        if (rc == 0) {
            Display_printf(displayHandle, 0, 0, "A[%d,%d,%d] G[%d,%d,%d] T:%d", 
                           sensor_data.accel_data[0], sensor_data.accel_data[1], sensor_data.accel_data[2],
                           sensor_data.gyro_data[0],  sensor_data.gyro_data[1],  sensor_data.gyro_data[2],
                           sensor_data.temp_data);
        } else {
            Display_printf(displayHandle, 0, 0, "Error reading sensor data");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

// Task Creation Function
void create_SPI_task(void)
{   
    xTaskCreate(SPI_task,       
                "Test SPI",             
                TASK_STACK_SIZE,   
                NULL,                     
                TASK_PRIORITY,     
                &SPI_task_handle); // Fixed parameter name to match declaration
}