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

// --- No-op function for APEX No-Motion Event ---
void apex_no_motion_noop(void)
{
    // This will now fire only when the IMU has been completely 
    // stationary for the duration of the timeout configured below.
    GPIO_toggle(CONFIG_LED_0_GPIO);
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
    if (spiHandle == NULL)
    {
        Display_printf(displayHandle, 0, 0, "SPI Init Failed!");
        while (1);
    }

    imu_dev.transport.read_reg = spi_read_reg;
    imu_dev.transport.write_reg = spi_write_reg;
    imu_dev.transport.sleep_us = delay_us;
    imu_dev.transport.serif_type = UI_SPI4;

    delay_us(3000); // Give IMU time to boot

    rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
    if (rc != 0 || whoami != INV_IMU_WHOAMI)
    {
        Display_printf(displayHandle, 0, 0, "IMU WHO_AM_I Failed!");
        while (1);
    }

    rc |= inv_imu_soft_reset(&imu_dev);

    /* --- 2. Sensor Configuration (100Hz & Low Noise) --- */
    rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
    rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_100_HZ);
    rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
    rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);

    /* --- 3. APEX Engine (EDMP) Configuration for NO-MOTION --- */
    
    // a. Power up SRAM and initialize baseline APEX
    rc |= inv_imu_edmp_init_apex(&imu_dev);

    // b. Set APEX Data Rate to 50Hz and recompute internal decimation
    rc |= inv_imu_edmp_set_frequency(&imu_dev, DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ);
    rc |= inv_imu_edmp_recompute_apex_decimation(&imu_dev);

    // c. Configure the NO-MOTION Cooldown / Timeout Parameter
    // Note: This MUST be done before enabling the EDMP algorithm
    inv_imu_edmp_apex_parameters_t apex_params;
    rc |= inv_imu_edmp_get_apex_parameters(&imu_dev, &apex_params);
    
    // Set how long the device must be completely still before firing.
    // The exact unit depends on the specific eMD driver version you have compiled, 
    // but it is usually based on APEX ODR ticks (50Hz = 20ms per tick) or directly in ms.
    // E.g., setting to 250 at 50Hz generally equals 5 seconds of required no-motion.
    apex_params.r2w_sleep_time_out = 500; 
    
    rc |= inv_imu_edmp_set_apex_parameters(&imu_dev, &apex_params);

    // d. Enable the Raise-to-Wake (R2W) feature (handles stationary "Sleep" detection)
    rc |= inv_imu_edmp_enable_r2w(&imu_dev);

    // e. Explicitly tell the APEX sub-system to trigger an interrupt internally on SLEEP (No-Motion)
    inv_imu_edmp_int_state_t apex_int_state;
    memset(&apex_int_state, 0, sizeof(apex_int_state));
    apex_int_state.INV_R2W_SLEEP = 1;
    rc |= inv_imu_edmp_set_config_int_apex(&imu_dev, &apex_int_state);

    // f. Turn on the entire EDMP Engine
    rc |= inv_imu_edmp_enable(&imu_dev);

    /* --- 4. Hardware Interrupt Pin Setup --- */
    inv_imu_int_pin_config_t int1_pin_config = {
        .int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH,
        .int_drive = INTX_CONFIG2_INTX_DRIVE_PP,
        .int_mode = INTX_CONFIG2_INTX_MODE_LATCH};

    rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int1_pin_config);

    // Route both the FIFO Watermark Threshold AND the Master EDMP Event to INT1
    inv_imu_int_state_t int1_state;
    memset(&int1_state, INV_IMU_DISABLE, sizeof(int1_state)); 
    int1_state.INV_FIFO_THS = INV_IMU_ENABLE;    
    int1_state.INV_EDMP_EVENT = INV_IMU_ENABLE;  
    rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int1_state);

    /* --- 5. FIFO Configuration (16-byte, WM=8) --- */
    inv_imu_fifo_config_t fifo_config;
    rc |= inv_imu_get_fifo_config(&imu_dev, &fifo_config);
    fifo_config.accel_en = INV_IMU_ENABLE;
    fifo_config.gyro_en = INV_IMU_ENABLE;
    fifo_config.hires_en = INV_IMU_DISABLE;
    fifo_config.fifo_wm_th = 8; 
    fifo_config.fifo_mode = FIFO_CONFIG0_FIFO_MODE_STREAM; 
    rc |= inv_imu_set_fifo_config(&imu_dev, &fifo_config);

    if (rc != 0)
    {
        Display_printf(displayHandle, 0, 0, "IMU Config Failed!");
        while (1);
    }
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

    GPIO_toggle(CONFIG_LED_0_GPIO);

    for (;;)
    {
        // Sleep until INT1 triggers the task notification
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        inv_imu_int_state_t int_status;
        int rc = inv_imu_get_int_status(&imu_dev, INV_IMU_INT1, &int_status);

        if (rc == 0)
        {
            // --- CHECK 1: Was it the FIFO Watermark Threshold? ---
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

            // --- CHECK 2: Was it an APEX / EDMP Event? ---
            if (int_status.INV_EDMP_EVENT)
            {
                inv_imu_edmp_int_state_t apex_status;
                int apex_rc = inv_imu_edmp_get_int_apex_status(&imu_dev, &apex_status);

                // If Raise-to-Wake determined the device is asleep (stationary)
                if (apex_rc == 0 && apex_status.INV_R2W_SLEEP)
                {
                    apex_no_motion_noop();
                }
            }
        }
    }
}

void INT1_callback_wakeup(uint_least8_t index)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (imuble_task_handle != NULL)
    {
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