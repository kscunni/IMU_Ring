#include "icm_func_impl.h"
#include "imu/inv_imu_defs.h"
#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_driver_advanced.h"
#include "imu/inv_imu_edmp.h"
#include <app_main.h>
#include "ti_drivers_config.h"
#include <common/Profiles/simple_gatt/simple_gatt_profile.h>

#include <FreeRTOS.h>

#include <string.h>
#include <task.h>
#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <unistd.h>

static inv_imu_device_t imu_dev;
int16_t ble_payload[48];
static Display_Handle displayHandle;
static TaskHandle_t imuble_task_handle = NULL;

#define TASK_STACK_SIZE 1024
#define TASK_PRIORITY 1

// --- Testing Constraints ---
#define WAKEUPS_FOR_5_SECONDS  62   // 12.5 wakeups/sec * 5 seconds

// --- State Variables ---
static uint32_t stillness_wakeups = 0;
static bool has_triggered_noop = false;

// --- No-op function for testing ---
void apex_no_motion_noop(void)
{
    // Print added so you can easily verify the timing on your serial monitor
    GPIO_write(CONFIG_LED_0_GPIO, CONFIG_GPIO_LED_ON);
}

void init_icm45605(void)
{
    int rc = 0;
    uint8_t whoami;

    /* --- 1. SPI & Driver Setup --- */
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 1000000;
    spiParams.dataSize = 8;
    spiParams.mode = SPI_CONTROLLER;

    spiHandle = SPI_open(CONFIG_SPI_0, &spiParams);
    if (spiHandle == NULL) { while (1); }

    imu_dev.transport.read_reg = spi_read_reg;
    imu_dev.transport.write_reg = spi_write_reg;
    imu_dev.transport.sleep_us = delay_us;
    imu_dev.transport.serif_type = UI_SPI4;

    delay_us(3000); 

    rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
    rc |= inv_imu_soft_reset(&imu_dev);

    /* --- 2. Sensor Configuration (100Hz & Low Noise) --- */
    rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
    rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_100_HZ);
    rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
    rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);

    /* --- 3. APEX Engine Configuration --- */
    rc |= inv_imu_edmp_init_apex(&imu_dev);
    rc |= inv_imu_edmp_set_frequency(&imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ);
    rc |= inv_imu_edmp_recompute_apex_decimation(&imu_dev);

    uint8_t wom_threshold = 13; // 50mg is highly sensitive
    
    rc |= inv_imu_adv_configure_wom(&imu_dev, 
                                    wom_threshold,  // X threshold
                                    wom_threshold,  // Y threshold
                                    wom_threshold,  // Z threshold
                                    TMST_WOM_CONFIG_WOM_INT_MODE_ORED, 
                                    TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);

    inv_imu_edmp_apex_parameters_t apex_params;
    rc |= inv_imu_edmp_get_apex_parameters(&imu_dev, &apex_params);
    apex_params.basicsmd_win = 1;       
    apex_params.basicsmd_win_wait = 1;
    rc |= inv_imu_edmp_set_apex_parameters(&imu_dev, &apex_params);

    // Enable SMD (Significant Motion Detection)
    rc |= inv_imu_edmp_enable_smd(&imu_dev);

    // Route SMD flag to internal registers only
    inv_imu_edmp_int_state_t apex_int_state;
    memset(&apex_int_state, 0, sizeof(apex_int_state));
    apex_int_state.INV_SMD = 1;
    rc |= inv_imu_edmp_set_config_int_apex(&imu_dev, &apex_int_state);

    rc |= inv_imu_edmp_enable(&imu_dev);

    /* --- 4. Hardware Interrupt Pin Setup (SILENT MOTION MODE) --- */
    inv_imu_int_pin_config_t int1_pin_config = {
        .int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH,
        .int_drive = INTX_CONFIG2_INTX_DRIVE_PP,
        .int_mode = INTX_CONFIG2_INTX_MODE_LATCH};

    rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int1_pin_config);

    // CRITICAL: Route ONLY the FIFO Watermark to physical pin INT1.
    inv_imu_int_state_t int1_state;
    memset(&int1_state, INV_IMU_DISABLE, sizeof(int1_state)); 
    int1_state.INV_FIFO_THS = INV_IMU_ENABLE;    
    rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int1_state);

    /* --- 5. FIFO Configuration --- */
    inv_imu_fifo_config_t fifo_config;
    rc |= inv_imu_get_fifo_config(&imu_dev, &fifo_config);
    fifo_config.accel_en = INV_IMU_ENABLE;
    fifo_config.gyro_en = INV_IMU_ENABLE;
    fifo_config.hires_en = INV_IMU_DISABLE;
    fifo_config.fifo_wm_th = 8; 
    fifo_config.fifo_mode = FIFO_CONFIG0_FIFO_MODE_STREAM; 
    rc |= inv_imu_set_fifo_config(&imu_dev, &fifo_config);
    if (rc != 0)
        while(1);
}

void ble_helper(char *Data)
{
    SimpleGattProfile_setParameter(SIMPLEGATTPROFILE_CHAR4, 96, Data);
}

static void imuble_task(void *pvParameters)
{
    Display_init();
    displayHandle = Display_open(Display_Type_UART, NULL);
    SPI_init();

    init_icm45605();

    for (;;)
    {
        // Wakes up exactly once every ~80ms when the FIFO watermark hits 8
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        inv_imu_int_state_t int_status;
        int rc = inv_imu_get_int_status(&imu_dev, INV_IMU_INT1, &int_status);

        if (rc == 0)
        {
            // ==========================================
            // 1. THE SILENT HARDWARE MOTION CHECK
            // ==========================================
            inv_imu_edmp_int_state_t apex_status;
            inv_imu_edmp_get_int_apex_status(&imu_dev, &apex_status);

            if (apex_status.INV_SMD)
            {
                // Movement occurred! Reset states.
                stillness_wakeups = 0;
                has_triggered_noop = false;
                GPIO_write(CONFIG_LED_0_GPIO, CONFIG_GPIO_LED_OFF);
            }
            else
            {
                // No movement caught by hardware since the last wakeup
                stillness_wakeups++;

                // Trigger exactly once when hitting 5 seconds
                if (stillness_wakeups >= WAKEUPS_FOR_5_SECONDS && !has_triggered_noop)
                {
                    apex_no_motion_noop();
                    has_triggered_noop = true; // Wait for motion before triggering again
                }
            }

            // ==========================================
            // 2. PROCESS FIFO DATA
            // ==========================================
            if (int_status.INV_FIFO_THS)
            {
                uint16_t frame_count = 0;
                rc |= inv_imu_get_frame_count(&imu_dev, &frame_count);

                for (uint16_t i = 0; i < 8; i++)
                {
                    inv_imu_fifo_data_t d;
                    rc |= inv_imu_get_fifo_frame(&imu_dev, &d);

                    if (rc == 0)
                    {
                        // Pack BLE payload raw
                        ble_payload[6*i+0] = d.byte_16.accel_data[0];
                        ble_payload[6*i+1] = d.byte_16.accel_data[1];
                        ble_payload[6*i+2] = d.byte_16.accel_data[2];
                        ble_payload[6*i+3] = d.byte_16.gyro_data[0];
                        ble_payload[6*i+4] = d.byte_16.gyro_data[1];
                        ble_payload[6*i+5] = d.byte_16.gyro_data[2];
                    }
                }
                
                if (ble_payload[0] != INVALID_VALUE_FIFO)
                {
                    BLEAppUtil_invokeFunction(ble_helper, (char *)ble_payload);
                }
            }
        }
    }
}

void INT1_callback_wakeup(uint_least8_t index)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (imuble_task_handle != NULL) {
        vTaskNotifyGiveFromISR(imuble_task_handle, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void create_imuble_task(void)
{
    xTaskCreate(imuble_task, "IMU_BLE", TASK_STACK_SIZE, NULL, TASK_PRIORITY,
                &imuble_task_handle);

    GPIO_setCallback(CONFIG_GPIO_INT1, INT1_callback_wakeup);
    GPIO_enableInt(CONFIG_GPIO_INT1);
}