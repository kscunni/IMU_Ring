
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h> // Added for serial console output
#include <unistd.h>             // For TI standard POSIX usleep
#include "ti_drivers_config.h"
#include "imu/inv_imu_driver.h" 
#include "imu/inv_imu_defs.h"

#include <FreeRTOS.h>
#include <task.h>
#include <string.h>
#include <app_main.h> 

#include "icm_func_impl.h" 

// Global handles

static inv_imu_device_t imu_dev;
static Display_Handle displayHandle;




void init_icm45605(void) {
    int rc = 0;
    uint8_t whoami;

    // GPIO_setConfig(6, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_HIGH | GPIO_CFG_OUT_STR_MED);
    // GPIO_write(6, 1);

    // 1. Initialize and open the TI SPI Driver
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 1000000;   // Start with 1 MHz
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

    inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
    inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_100_HZ);

    // 6. Turn on Accelerometer and Gyroscope in Low Noise (LN) mode
    rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
    rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);

    // Configure INT1 to be Active High, Push-Pull, and clear on data read (Latched)
inv_imu_int_pin_config_t int1_config;

// 1 = Active High (idles low, pulls high), 0 = Active Low
int1_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH; 

// 1 = Push-Pull (no external resistor needed), 0 = Open Drain
int1_config.int_drive = INTX_CONFIG2_INTX_DRIVE_PP;       

// 1 = Latched (stays high until SPI read), 0 = Pulsed (short blip)
int1_config.int_mode = INTX_CONFIG2_INTX_MODE_LATCH;      


// Apply the hardware configuration
inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int1_config);

// 1. Create and zero-initialize the interrupt state structure
    inv_imu_int_state_t int1_state = {0}; 

    // 2. Enable only the Data Ready (DRDY) interrupt routing
    int1_state.INV_UI_DRDY = 1;

    // 3. Apply the routing configuration to INT1
    inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int1_state);


    if (rc == 0) {
        Display_printf(displayHandle, 0, 0, "ICM45605 Initialized Successfully!");
    } else {
        Display_printf(displayHandle, 0, 0, "ICM45605 Initialization sequence returned error: %d", rc);
    }
}


// Task Configuration 
#define TASK_STACK_SIZE 512 
#define TASK_PRIORITY   1   
static TaskHandle_t imuble_task_handle = NULL;

// Task
static void imuble_task(void *pvParameters)
{
    inv_imu_sensor_data_t sensor_data;
    
    // Initialize UART Display for logging
    Display_init();
    displayHandle = Display_open(Display_Type_UART, NULL);
    
    SPI_init();
    
    // Initialize the IMU
    init_icm45605();

    long last_ticks = xTaskGetTickCount();
    long ticks = last_ticks;
    
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ticks = xTaskGetTickCount();
        // Read the 3-axis accel, 3-axis gyro, and temperature
        int rc = inv_imu_get_register_data(&imu_dev, &sensor_data);

        Display_printf(displayHandle, 0, 0, "%d\n", (ticks - last_ticks));
        last_ticks = ticks;
        
        // if (rc == 0) {
        //     Display_printf(displayHandle, 0, 0, "A[%d,%d,%d] G[%d,%d,%d] T:%d", 
        //                    sensor_data.accel_data[0], sensor_data.accel_data[1], sensor_data.accel_data[2],
        //                    sensor_data.gyro_data[0],  sensor_data.gyro_data[1],  sensor_data.gyro_data[2],
        //                    sensor_data.temp_data);
        // } else {
        //     Display_printf(displayHandle, 0, 0, "Error reading sensor data");
        // }
        
        // vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

void INT1_callback_wakeup(uint_least8_t index)
{
    // Required by FreeRTOS to track if waking the task requires an immediate context switch
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Send a notification to the sleeping task
    if (imuble_task_handle != NULL) {
        vTaskNotifyGiveFromISR(imuble_task_handle, &xHigherPriorityTaskWoken);
    }

    // Yield execution if the woken task has a higher priority than the currently running task
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Task Creation Function
void create_imuble_task(void)
{   
    xTaskCreate(imuble_task,       
                "Test imuble",             
                TASK_STACK_SIZE,   
                NULL,                     
                TASK_PRIORITY,     
                &imuble_task_handle); // Fixed parameter name to match declaration

    // Enable interrupt to wake this task
    GPIO_setCallback(CONFIG_GPIO_INT1, INT1_callback_wakeup);
    GPIO_enableInt(CONFIG_GPIO_INT1);
}