#include "icm_func_impl.h"
#include "imu/inv_imu_defs.h"
#include "imu/inv_imu_driver.h"

#include "ti_drivers_config.h"
#include <FreeRTOS.h>
#include <app_main.h>
#include <string.h>
#include <task.h>
#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <unistd.h>
#include <common/Profiles/simple_gatt/simple_gatt_profile.h>
static inv_imu_device_t imu_dev;
inv_imu_sensor_data_t sensor_data;
static Display_Handle displayHandle;
static TaskHandle_t imuble_task_handle = NULL;
// const int32_t gyro_offset[3] = {-5, 1, -1};
#define TASK_STACK_SIZE 1024
#define TASK_PRIORITY 1

void init_icm45605(void)
{
    int rc = 0;
    uint8_t whoami;

    // SPI
    SPI_Params spiParams;
    SPI_Params_init(&spiParams);
    spiParams.bitRate = 1000000; // maybe change later
    spiParams.dataSize = 8;
    spiParams.mode = SPI_CONTROLLER;

    spiHandle = SPI_open(CONFIG_SPI_0, &spiParams);
    if (spiHandle == NULL)
    {
        Display_printf(displayHandle, 0, 0, "SPI Init Failed!");
        while (1);
    }

    // link to implementations
    imu_dev.transport.read_reg = spi_read_reg;
    imu_dev.transport.write_reg = spi_write_reg;
    imu_dev.transport.sleep_us = delay_us;
    imu_dev.transport.serif_type = UI_SPI4;

    delay_us(3000);

    rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
    if (rc != 0 || whoami != INV_IMU_WHOAMI)
    {
        Display_printf(displayHandle, 0, 0, "IMU WHO_AM_I Failed!");
        while (1);
    }

    rc |= inv_imu_soft_reset(&imu_dev);

    // set 100 Hz Low Noise mode
    inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_100_HZ);
    inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_100_HZ);
    rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
    rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);

    // rc |= inv_imu_set_gyro_offset(&imu_dev, gyro_offset);

    // set up interrupt on ready
    inv_imu_int_pin_config_t int1_config = {
        .int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH,
        .int_drive = INTX_CONFIG2_INTX_DRIVE_PP,
        .int_mode = INTX_CONFIG2_INTX_MODE_LATCH};
    inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int1_config);

    inv_imu_int_state_t int1_state = {0};
    int1_state.INV_UI_DRDY = 1;
    inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int1_state);
}

// Turns out you need to pass in a function pointer to
// BLEAppUtil_invokeFunction() for it to work
void ble_helper(char *Data)
{
    // setting this parameter causes a "notify"
    SimpleGattProfile_setParameter(SIMPLEGATTPROFILE_CHAR4, 12, Data);
}

static void imuble_task(void *pvParameters)
{

    // for temporary serial prints for debug
    Display_init();
    displayHandle = Display_open(Display_Type_UART, NULL);
    SPI_init();

    init_icm45605();

    // long last_ticks = xTaskGetTickCount();
    // long ticks = last_ticks;


    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // ticks = xTaskGetTickCount();
        int rc = inv_imu_get_register_data(&imu_dev, &sensor_data);

        if (rc == 0)
        {
            BLEAppUtil_invokeFunction(ble_helper, (char *)&sensor_data);
            // Display_printf(displayHandle, 0, 0, "%d\n", (ticks -
            // last_ticks)); last_ticks = ticks;
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

    // attach wakeup interrupt
    GPIO_setCallback(CONFIG_GPIO_INT1, INT1_callback_wakeup);
    GPIO_enableInt(CONFIG_GPIO_INT1);
}