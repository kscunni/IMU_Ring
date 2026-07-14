#include "icm_func_impl.h"
#include "imu/inv_imu_defs.h"
#include "imu/inv_imu_driver.h"
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

#define IMU_EVENT_DATA_READY  (1 << 0)  // Bit 0: Hardware interrupt from IMU
#define IMU_EVENT_SHUTDOWN    (1 << 1)  // Bit 1: Command from control task
#define CTRL_EVENT_IMU_SHUTDOWN_ACK  (1 << 0)

static inv_imu_device_t imu_dev;
int16_t ble_payload[57];
static Display_Handle displayHandle;
TaskHandle_t imuble_task_handle = NULL;
extern TaskHandle_t button_task_handle;

int16_t goffsetx = -46, goffsety = 8, goffsetz = -12;

volatile int8_t event_marker;

#define TASK_STACK_SIZE 1024
#define TASK_PRIORITY 1

void shutdown_icm45605(void);

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

    // Link transport functions to the implementations in icm_func_impl.c
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

    rc |= inv_imu_set_gyro_offset(&imu_dev, goffsetx, goffsety, goffsetz);

    /* --- 3. Interrupt Pin Setup --- */
    // Using PULSE mode is recommended for FIFO thresholds
    inv_imu_int_pin_config_t int1_pin_config = {
        .int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH,
        .int_drive = INTX_CONFIG2_INTX_DRIVE_PP,
        .int_mode = INTX_CONFIG2_INTX_MODE_LATCH};

    rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int1_pin_config);

    // Route the FIFO Watermark Threshold to INT1
    inv_imu_int_state_t int1_state;
    memset(&int1_state, INV_IMU_DISABLE, sizeof(int1_state)); // Clear all
    int1_state.INV_FIFO_THS = INV_IMU_ENABLE; // Enable FIFO Threshold
    rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int1_state);

    /* --- 4. FIFO Configuration (16-byte, WM=8) --- */
    inv_imu_fifo_config_t fifo_config;
    rc |= inv_imu_get_fifo_config(&imu_dev, &fifo_config);
    fifo_config.accel_en = INV_IMU_ENABLE;
    fifo_config.gyro_en = INV_IMU_ENABLE;
    fifo_config.hires_en =
        INV_IMU_DISABLE;        // Disabling hires results in 16-byte packets
    fifo_config.fifo_wm_th = 8; // Trigger interrupt after 8 packets
    fifo_config.fifo_mode =
        FIFO_CONFIG0_FIFO_MODE_STREAM; // Stream overrides oldest data if full
    rc |= inv_imu_set_fifo_config(&imu_dev, &fifo_config);

    if (rc != 0)
    {
        Display_printf(displayHandle, 0, 0, "IMU Config Failed!");
        while (1);
    }
}

void shutdown_icm45605(void)
{
    int rc = 0;

    /* --- 1. Disable Interrupts --- */
    // Clear all interrupt routings to prevent the IMU from asserting INT1
    inv_imu_int_state_t int1_state;
    memset(&int1_state, INV_IMU_DISABLE, sizeof(int1_state)); 
    rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int1_state);

    /* --- 2. Disable FIFO --- */
    // Stop routing Accel and Gyro data to the FIFO
    inv_imu_fifo_config_t fifo_config;
    rc |= inv_imu_get_fifo_config(&imu_dev, &fifo_config);
    fifo_config.accel_en = INV_IMU_DISABLE;
    fifo_config.gyro_en  = INV_IMU_DISABLE;
    rc |= inv_imu_set_fifo_config(&imu_dev, &fifo_config);

    /* --- 3. Power Down Sensors (Sleep Mode) --- */
    // Setting both Accel and Gyro to OFF drops the ICM into its lowest power state
    rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_OFF);
    rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_OFF);

    if (rc != 0)
    {
        Display_printf(displayHandle, 0, 0, "IMU Shutdown Failed!");
    }
}

// Turns out you need to pass in a function pointer to
// BLEAppUtil_invokeFunction() for it to work
void ble_helper(char *Data)
{
    // setting this parameter causes a "notify" (Sending 12 bytes: 3 Accel + 3
    // Gyro)
    SimpleGattProfile_setParameter(SIMPLEGATTPROFILE_CHAR4, SIMPLEGATTPROFILE_CHAR4_LEN, Data);
}

static void imuble_task(void *pvParameters)
{

    // for temporary serial prints for debug
    Display_init();
    displayHandle = Display_open(Display_Type_UART, NULL);
    SPI_init();

    init_icm45605();

    uint32_t notifiedValue;

    for (;;)
    {
        // Sleep until INT1 triggers the task notification
        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        xTaskNotifyWait(
            0x00,             // Don't clear any bits on entry
            0xFFFFFFFF,       // Clear all bits on exit so we start fresh
            &notifiedValue,   // Store the bits that woke us up here
            portMAX_DELAY     // Wait indefinitely
        );

        // --- Event 1: Control Task sent Shutdown Command ---
        if (notifiedValue & IMU_EVENT_SHUTDOWN)
        {
            shutdown_icm45605();

            xTaskNotify(button_task_handle, CTRL_EVENT_IMU_SHUTDOWN_ACK, eSetBits);

            // vTaskSuspend(NULL); 
        }

        // --- Event 2: IMU Hardware Interrupt (FIFO Ready) ---
        if (notifiedValue & IMU_EVENT_DATA_READY)
        {


            inv_imu_int_state_t int_status;
            int rc = inv_imu_get_int_status(&imu_dev, INV_IMU_INT1, &int_status);

            // If the interrupt was caused by the FIFO Watermark Threshold
            if (rc == 0 && int_status.INV_FIFO_THS)
            {

                // uint16_t frame_count = 0;
                // rc |= inv_imu_get_frame_count(&imu_dev, &frame_count);

                // Loop through all currently available frames in the FIFO
                for (uint16_t i = 0; i < 8; i++)
                {
                    inv_imu_fifo_data_t d;
                    rc |= inv_imu_get_fifo_frame(&imu_dev, &d);

                    if (rc == 0)
                    {
                        // Create a 12-byte payload struct mapping precisely to what
                        // BLE needs 6 x 16-bit values (Accel X/Y/Z, Gyro X/Y/Z) =
                        // 12 bytes
                        
                        ble_payload[7*i+0] = d.byte_16.accel_data[0];
                        ble_payload[7*i+1] = d.byte_16.accel_data[1];
                        ble_payload[7*i+2] = d.byte_16.accel_data[2];
                        ble_payload[7*i+3] = d.byte_16.gyro_data[0];
                        ble_payload[7*i+4] = d.byte_16.gyro_data[1];
                        ble_payload[7*i+5] = d.byte_16.gyro_data[2];
                        ble_payload[7*i+6] = d.byte_16.timestamp;

                        
                    }
                }
                ble_payload[56] = event_marker; // little endian, so this is correct.
                event_marker = 0;
                // Discard dummy startup values that occur when IMU is just
                // turned on
                if (ble_payload[0] != INVALID_VALUE_FIFO)
                {
                    BLEAppUtil_invokeFunction(ble_helper,
                                                (char *)ble_payload);
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
        // vTaskNotifyGiveFromISR(imuble_task_handle, &xHigherPriorityTaskWoken);
        xTaskNotifyFromISR(imuble_task_handle, 
                   IMU_EVENT_DATA_READY, 
                   eSetBits, 
                   &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void create_imuble_task(void)
{
    xTaskCreate(imuble_task, "IMU_BLE", TASK_STACK_SIZE, NULL, TASK_PRIORITY,
                &imuble_task_handle);

    // attach wakeup interrupt
    GPIO_setCallback(CONFIG_GPIO_INT1, INT1_callback_wakeup);
    GPIO_enableInt(CONFIG_GPIO_INT1);
}