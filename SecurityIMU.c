#include "icm_func_impl.h"
#include "imu/inv_imu_defs.h"
#include "imu/inv_imu_driver.h"
#include <app_main.h>
#include "ti_drivers_config.h"
#include <common/Profiles/simple_gatt/simple_gatt_profile.h>

#include <FreeRTOS.h>
#include <string.h>
#include <task.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <unistd.h>
#include <stdio.h>
#include <ti/drivers/UART2.h>
#include <stdlib.h>

// Shared UART handles/buffers for passkey testing
extern UART2_Handle uartHandle;
extern volatile int8_t uart_digit_buffer;


// ============================================================================
// Axis Mapping & Sign Configuration (+Y = Forward, +Z = Up, +X = Right)
// ============================================================================
#define AXIS_FWD     1   // Y-axis: Forward (+Y) / Backward (-Y)
#define AXIS_LAT     0   // X-axis: Right (+X) / Left (-X)
#define AXIS_VERT    2   // Z-axis: Up (+Z) / Down (-Z)

#define AXIS_ROLL    1   // Y-axis: Wrist Roll around the Forward axis
#define AXIS_PITCH   0   // X-axis: Wrist Pitch around the Left/Right axis

#define SIGN_FWD     ( 1)  // +Y is Forward (5), -Y is Backward (6)
#define SIGN_LAT     (-1)  // -X is Left (2), +X is Right (3)
#define SIGN_VERT    ( 1)  // +Z is Up (4), -Z is Down (1)
#define SIGN_ROLL    ( 1)  // +Gy is Clockwise (8), -Gy is Counter-Clockwise (7)
#define SIGN_PITCH   ( 1)  // +Gx is Point Up (9), -Gx is Point Down (0)

// ============================================================================
// Thresholds & Timing Constants (100Hz = 10ms per sample)
// ============================================================================
#define ACCEL_QUIET_TH       300   // Max accel variation when hand is still
#define GYRO_QUIET_TH        400   // Max gyro rate when hand is still

// Outgoing stroke thresholds
#define ACCEL_OUT_TH         900   // Initial linear impulse required (1..6)
#define GYRO_OUT_TH          1500  // Initial rotation rate required (7,8,9,0)

// Return/Reversal stroke thresholds (~60% of outgoing to prevent false rejections)
#define ACCEL_RET_TH         600   // Turnaround acceleration required in opposite direction
#define GYRO_RET_TH          1100  // Return rotation rate required in opposite direction
#define POSTURE_RETURN_TOL   700   // Max gravity difference between start and end posture

// Timing windows (in 10ms samples at 100Hz)
#define SETTLE_SAMPLES       25    // 250ms stillness to lock initial baseline
#define OUTGOING_WINDOW      16    // 160ms to capture the initial outgoing impulse
#define RETURN_TIMEOUT       65    // 650ms max time to see the return stroke
#define COOLDOWN_SAMPLES     25    // 250ms stillness after return stroke completes

typedef enum {
    GESTURE_STATE_SETTLE = 0,
    GESTURE_STATE_ARMED,
    GESTURE_STATE_OUTGOING,
    GESTURE_STATE_WAIT_RETURN,
    GESTURE_STATE_COOLDOWN
} gesture_state_t;

static gesture_state_t g_state = GESTURE_STATE_SETTLE;
static int32_t  accel_base[3] = {0, 0, 0};
static uint16_t state_counter = 0;

// Outgoing stroke tracking
static int16_t peak_da[3]   = {0, 0, 0};
static int16_t peak_gyro[3] = {0, 0, 0};

// Locked parameters for verifying the return stroke
static int8_t  pending_digit     = -1;
static bool    locked_is_gyro    = false;
static uint8_t locked_axis       = 0;
static int8_t  expected_ret_sign = 0; // +1 or -1 on the raw sensor axis

static inline int16_t abs16(int16_t x)
{
    return (x < 0) ? -x : x;
}

static void reset_gesture_detector(void)
{
    g_state = GESTURE_STATE_SETTLE;
    state_counter = 0;
    pending_digit = -1;
    accel_base[0] = 0;
    accel_base[1] = 0;
    accel_base[2] = 0;

    if (uartHandle != NULL)
    {
        const char prompt[] = "\r\n[IMU] Hold hand forward & still to arm gesture...\r\n";
        size_t bw;
        UART2_write(uartHandle, prompt, sizeof(prompt) - 1, &bw);
    }
}

/**
 * @brief Evaluates the outgoing impulse, selects the candidate digit (0-9),
 *        and locks the axis and opposite sign required for the return motion.
 */
static int8_t lock_outgoing_stroke(void)
{
    int16_t raw_roll  = peak_gyro[AXIS_ROLL];
    int16_t raw_pitch = peak_gyro[AXIS_PITCH];

    int16_t abs_roll  = abs16(raw_roll);
    int16_t abs_pitch = abs16(raw_pitch);

    // 1. Check Rotational Gestures First (7, 8, 9, 0)
    if (abs_roll > GYRO_OUT_TH || abs_pitch > GYRO_OUT_TH)
    {
        locked_is_gyro = true;

        if (abs_roll > abs_pitch)
        {
            locked_axis = AXIS_ROLL;
            expected_ret_sign = (raw_roll > 0) ? -1 : 1; // Return must be opposite raw sign
            int16_t signed_roll = raw_roll * SIGN_ROLL;
            return (signed_roll > 0) ? 8 : 7; // 8 = CW, 7 = CCW
        }
        else
        {
            locked_axis = AXIS_PITCH;
            expected_ret_sign = (raw_pitch > 0) ? -1 : 1;
            int16_t signed_pitch = raw_pitch * SIGN_PITCH;
            return (signed_pitch > 0) ? 9 : 0; // 9 = Point Up, 0 = Point Down
        }
    }

    // 2. Check Linear Translations (1..6)
    int16_t abs_fwd  = abs16(peak_da[AXIS_FWD]);
    int16_t abs_lat  = abs16(peak_da[AXIS_LAT]);
    int16_t abs_vert = abs16(peak_da[AXIS_VERT]);

    int16_t max_lin = abs_vert;
    if (abs_lat > max_lin) max_lin = abs_lat;
    if (abs_fwd > max_lin) max_lin = abs_fwd;

    if (max_lin < ACCEL_OUT_TH)
    {
        return -1; // Outgoing motion too weak
    }

    locked_is_gyro = false;

    if (max_lin == abs_vert)
    {
        locked_axis = AXIS_VERT;
        expected_ret_sign = (peak_da[AXIS_VERT] > 0) ? -1 : 1;
        int16_t signed_vert = peak_da[AXIS_VERT] * SIGN_VERT;
        return (signed_vert > 0) ? 4 : 1; // 4 = Up, 1 = Down
    }
    else if (max_lin == abs_lat)
    {
        locked_axis = AXIS_LAT;
        expected_ret_sign = (peak_da[AXIS_LAT] > 0) ? -1 : 1;
        int16_t signed_lat = peak_da[AXIS_LAT] * SIGN_LAT;
        return (signed_lat > 0) ? 2 : 3; // 2 = Left, 3 = Right
    }
    else
    {
        locked_axis = AXIS_FWD;
        expected_ret_sign = (peak_da[AXIS_FWD] > 0) ? -1 : 1;
        int16_t signed_fwd = peak_da[AXIS_FWD] * SIGN_FWD;
        return (signed_fwd > 0) ? 5 : 6; // 5 = Forward, 6 = Backward
    }
}

static int8_t detect_gesture_digit(const int16_t accel[3], const int16_t gyro[3], uint16_t timestamp)
{
    (void)timestamp;

    // Linear acceleration relative to resting baseline
    int16_t da[3];
    da[0] = accel[0] - (int16_t)accel_base[0];
    da[1] = accel[1] - (int16_t)accel_base[1];
    da[2] = accel[2] - (int16_t)accel_base[2];

    bool is_quiet = (abs16(gyro[0]) < GYRO_QUIET_TH) &&
                    (abs16(gyro[1]) < GYRO_QUIET_TH) &&
                    (abs16(gyro[2]) < GYRO_QUIET_TH) &&
                    (abs16(da[0])   < ACCEL_QUIET_TH) &&
                    (abs16(da[1])   < ACCEL_QUIET_TH) &&
                    (abs16(da[2])   < ACCEL_QUIET_TH);

    switch (g_state)
    {
        case GESTURE_STATE_SETTLE:
        {
            if (state_counter == 0)
            {
                accel_base[0] = accel[0];
                accel_base[1] = accel[1];
                accel_base[2] = accel[2];
                state_counter = 1;
            }
            else
            {
                accel_base[0] += (accel[0] - accel_base[0]) >> 2;
                accel_base[1] += (accel[1] - accel_base[1]) >> 2;
                accel_base[2] += (accel[2] - accel_base[2]) >> 2;

                if (is_quiet)
                {
                    if (++state_counter >= SETTLE_SAMPLES)
                    {
                        g_state = GESTURE_STATE_ARMED;
                        state_counter = 0;
                    }
                }
                else
                {
                    state_counter = 1;
                }
            }
            break;
        }

        case GESTURE_STATE_ARMED:
        {
            // Slowly track posture drift ONLY while armed and quiet
            if (is_quiet)
            {
                accel_base[0] += (accel[0] - accel_base[0]) >> 4;
                accel_base[1] += (accel[1] - accel_base[1]) >> 4;
                accel_base[2] += (accel[2] - accel_base[2]) >> 4;
            }

            bool motion_started = (abs16(da[0])   > (ACCEL_OUT_TH / 2)) ||
                                  (abs16(da[1])   > (ACCEL_OUT_TH / 2)) ||
                                  (abs16(da[2])   > (ACCEL_OUT_TH / 2)) ||
                                  (abs16(gyro[0]) > (GYRO_OUT_TH / 2))  ||
                                  (abs16(gyro[1]) > (GYRO_OUT_TH / 2));

            if (motion_started)
            {
                for (int k = 0; k < 3; k++)
                {
                    peak_da[k]   = da[k];
                    peak_gyro[k] = gyro[k];
                }
                state_counter = 1;
                g_state = GESTURE_STATE_OUTGOING;
            }
            break;
        }

        case GESTURE_STATE_OUTGOING:
        {
            // Capture peak values of the initial outgoing stroke (160ms)
            for (int k = 0; k < 3; k++)
            {
                if (abs16(da[k]) > abs16(peak_da[k]))     peak_da[k]   = da[k];
                if (abs16(gyro[k]) > abs16(peak_gyro[k])) peak_gyro[k] = gyro[k];
            }

            if (++state_counter >= OUTGOING_WINDOW)
            {
                pending_digit = lock_outgoing_stroke();
                state_counter = 0;

                if (pending_digit >= 0)
                {
                    // Valid outgoing movement! Now wait for the reversal/return stroke
                    g_state = GESTURE_STATE_WAIT_RETURN;
                }
                else
                {
                    // False start / weak bump -> wait for stillness and re-settle
                    g_state = GESTURE_STATE_SETTLE;
                }
            }
            break;
        }

        case GESTURE_STATE_WAIT_RETURN:
        {
            bool return_detected = false;

            if (locked_is_gyro)
            {
                // Gyro: Check if the locked rotation axis reverses past GYRO_RET_TH
                int16_t val = gyro[locked_axis] * expected_ret_sign;
                if (val > GYRO_RET_TH)
                {
                    return_detected = true;
                }
            }
            else
            {
                // Accel: Check if the locked linear axis reverses past ACCEL_RET_TH
                int16_t val = da[locked_axis] * expected_ret_sign;
                if (val > ACCEL_RET_TH)
                {
                    return_detected = true;
                }
            }

            if (return_detected)
            {
                // Both outgoing and return strokes verified! Wait for hand to settle
                state_counter = 0;
                g_state = GESTURE_STATE_COOLDOWN;
            }
            else if (++state_counter >= RETURN_TIMEOUT)
            {
                // Timed out waiting for return motion (one-way movement rejected!)
                if (uartHandle != NULL)
                {
                    char msg[] = "Rejected (no return stroke)\r\n";
                    size_t bw;
                    UART2_write(uartHandle, msg, sizeof(msg) - 1, &bw);
                }
                pending_digit = -1;
                state_counter = 0;
                g_state = GESTURE_STATE_SETTLE;
            }
            break;
        }

        case GESTURE_STATE_COOLDOWN:
        {
            // Wait for hand to stop moving after completing the return stroke
            if (is_quiet)
            {
                if (++state_counter >= COOLDOWN_SAMPLES)
                {
                    // Verify final posture returned close to the initial forward orientation
                    bool posture_restored = (abs16(da[0]) < POSTURE_RETURN_TOL) &&
                                            (abs16(da[1]) < POSTURE_RETURN_TOL) &&
                                            (abs16(da[2]) < POSTURE_RETURN_TOL);

                    int8_t result = -1;
                    if (posture_restored)
                    {
                        result = pending_digit;
                        if (uartHandle != NULL && result >= 0)
                        {
                            char dbg[64];
                            size_t bw;
                            int len = snprintf(dbg, sizeof(dbg),
                                               "Accepted Digit: %d\r\n", result);
                            if (len > 0) UART2_write(uartHandle, dbg, (size_t)len, &bw);
                        }
                    }
                    else if (uartHandle != NULL)
                    {
                        char msg[] = "Rejected (did not return to neutral posture)\r\n";
                        size_t bw;
                        UART2_write(uartHandle, msg, sizeof(msg) - 1, &bw);
                    }

                    pending_digit = -1;
                    state_counter = 0;
                    // Re-settle baseline for the next gesture
                    g_state = GESTURE_STATE_SETTLE;
                    return result;
                }
            }
            else
            {
                state_counter = 0;
            }
            break;
        }
    }

    return -1;
}

#define PASSKEY_LENGTH  6
#define TASK_STACK_SIZE 1024
#define TASK_PRIORITY   1

typedef enum {
    IMU_MODE_NORMAL_STREAM = 0,
    IMU_MODE_PASSKEY_ENTRY
} imu_mode_t;

static inv_imu_device_t imu_dev;
int16_t ble_payload[57];
TaskHandle_t imuble_task_handle = NULL;

int16_t goffsetx = -46, goffsety = 8, goffsetz = -12;
volatile int8_t event_marker;

// Passkey state variables
static imu_mode_t current_imu_mode = IMU_MODE_NORMAL_STREAM;
static uint32_t passkey_accumulator = 0;
static uint8_t  digits_collected = 0;
volatile uint32_t completed_passkey = 0;

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
    inv_imu_int_pin_config_t int1_pin_config = {
        .int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH,
        .int_drive = INTX_CONFIG2_INTX_DRIVE_PP,
        .int_mode = INTX_CONFIG2_INTX_MODE_LATCH};

    rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int1_pin_config);

    // Route the FIFO Watermark Threshold to INT1
    inv_imu_int_state_t int1_state;
    memset(&int1_state, INV_IMU_DISABLE, sizeof(int1_state));
    int1_state.INV_FIFO_THS = INV_IMU_ENABLE;
    rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int1_state);

    /* --- 4. FIFO Configuration (16-byte, WM=8) --- */
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
        while (1);
    }
}

void shutdown_icm45605(void)
{
    int rc = 0;

    /* --- 1. Disable Interrupts --- */
    inv_imu_int_state_t int1_state;
    memset(&int1_state, INV_IMU_DISABLE, sizeof(int1_state));
    rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int1_state);

    /* --- 2. Disable FIFO --- */
    inv_imu_fifo_config_t fifo_config;
    rc |= inv_imu_get_fifo_config(&imu_dev, &fifo_config);
    fifo_config.accel_en = INV_IMU_DISABLE;
    fifo_config.gyro_en  = INV_IMU_DISABLE;
    rc |= inv_imu_set_fifo_config(&imu_dev, &fifo_config);

    /* --- 3. Power Down Sensors (Sleep Mode) --- */
    rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_OFF);
    rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_OFF);

    (void)rc;
}

void ble_helper(char *Data)
{
    SimpleGattProfile_setParameter(SIMPLEGATTPROFILE_CHAR4, SIMPLEGATTPROFILE_CHAR4_LEN, Data);
}


// // Track dummy latest IMU values so the compiler doesn't optimize out parameters
// // and so you can inspect live sensor readings at a breakpoint
// static volatile int16_t debug_last_ax = 0;
// static volatile int16_t debug_last_gx = 0;

// static void reset_gesture_detector(void)
// {
//     // Clear any stale key press before starting
//     uart_digit_buffer = -1;

//     if (uartHandle != NULL)
//     {
//         const char prompt[] = "\r\n[IMU Task] Passkey mode active! Enter 6 digits: ";
//         size_t bytesWritten;
//         UART2_write(uartHandle, prompt, sizeof(prompt) - 1, &bytesWritten);
//     }
// }

// static int8_t detect_gesture_digit(const int16_t accel[3], const int16_t gyro[3], uint16_t timestamp)
// {
//     int8_t detected_digit = -1;

//     // Store latest X-axis sample so debugger breakpoints can inspect live IMU data
//     debug_last_ax = accel[0];
//     debug_last_gx = gyro[0];
//     (void)timestamp;

//     // Check if a single digit (0-9) arrived over UART
//     int8_t incoming = uart_digit_buffer;
//     if (incoming >= 0 && incoming <= 9)
//     {
//         detected_digit = incoming;
//         uart_digit_buffer = -1; // Mark consumed

//         // Echo the accepted single digit to the terminal
//         if (uartHandle != NULL)
//         {
//             char echoChar = (char)('0' + detected_digit);
//             size_t bytesWritten;
//             UART2_write(uartHandle, &echoChar, 1, &bytesWritten);
//         }
//     }

//     return detected_digit;
// }

// /**
//  * @brief Reset gesture recognition state machine before a new passkey attempt
//  */
// static void reset_gesture_detector(void)
// {
//     // TODO: Clear gesture window buffers, state flags, or debounce timers here
// }

// /**
//  * @brief Skeleton function to evaluate a single IMU frame and return a digit (0-9)
//  *
//  * @param accel     3-axis accelerometer array [X, Y, Z]
//  * @param gyro      3-axis gyroscope array [X, Y, Z]
//  * @param timestamp IMU frame timestamp
//  * @return int8_t   Recognized digit (0-9), or -1 if no complete gesture detected yet
//  */
// static int8_t detect_gesture_digit(const int16_t accel[3], const int16_t gyro[3], uint16_t timestamp)
// {
//     int8_t detected_digit = -1;

//     // TODO: Implement gesture classification logic here.
//     // When a complete gesture is recognized, set detected_digit to 0..9.

//     return detected_digit;
// }

static void imuble_task(void *pvParameters)
{
    SPI_init();
    init_icm45605();

    uint32_t notifiedValue;

    for (;;)
    {
        xTaskNotifyWait(
            0x00,             // Don't clear any bits on entry
            0xFFFFFFFF,       // Clear all bits on exit
            &notifiedValue,   // Store the bits that woke us up
            portMAX_DELAY     // Wait indefinitely
        );

        // --- Event 1: Control Task sent Shutdown Command ---
        if (notifiedValue & IMU_EVENT_SHUTDOWN)
        {
            shutdown_icm45605();
            xTaskNotify(button_task_handle, CTRL_EVENT_IMU_SHUTDOWN_ACK, eSetBits);
            continue; // Do not process any remaining FIFO events after shutdown
        }

        // --- Event 2: BLE Stack requested Passkey Entry ---
        if (notifiedValue & IMU_EVENT_START_PASSKEY)
        {
            current_imu_mode = IMU_MODE_PASSKEY_ENTRY;
            passkey_accumulator = 0;
            digits_collected = 0;
            reset_gesture_detector();
        }

        // --- Event 3: BLE Pairing finished or timed out ---
        if (notifiedValue & IMU_EVENT_CANCEL_PASSKEY)
        {
            current_imu_mode = IMU_MODE_NORMAL_STREAM;
            passkey_accumulator = 0;
            digits_collected = 0;
            reset_gesture_detector();
        }

        // --- Event 4: IMU Hardware Interrupt (FIFO Ready) ---
        if (notifiedValue & IMU_EVENT_DATA_READY)
        {
            inv_imu_int_state_t int_status;
            int rc = inv_imu_get_int_status(&imu_dev, INV_IMU_INT1, &int_status);

            if (rc == 0 && int_status.INV_FIFO_THS)
            {
                for (uint16_t i = 0; i < 8; i++)
                {
                    inv_imu_fifo_data_t d;
                    rc |= inv_imu_get_fifo_frame(&imu_dev, &d);

                    if (rc == 0 && d.byte_16.accel_data[0] != INVALID_VALUE_FIFO)
                    {
                        if (current_imu_mode == IMU_MODE_PASSKEY_ENTRY)
                        {
                            int8_t digit = detect_gesture_digit(
                                d.byte_16.accel_data,
                                d.byte_16.gyro_data,
                                d.byte_16.timestamp
                            );

                            if (digit >= 0 && digit <= 9)
                            {
                                // Accumulate base-10 passkey (e.g., 1 -> 12 -> 123...)
                                passkey_accumulator = (passkey_accumulator * 10U) + (uint32_t)digit;
                                digits_collected++;

                                // Optional: Toggle LED so user knows a gesture digit was registered
                                GPIO_toggle(CONFIG_LED_0_GPIO);

                                if (digits_collected >= PASSKEY_LENGTH)
                                {
                                    completed_passkey = passkey_accumulator;

                                    // Reset state and return to normal streaming mode
                                    current_imu_mode = IMU_MODE_NORMAL_STREAM;
                                    passkey_accumulator = 0;
                                    digits_collected = 0;

                                    // Passes pData = NULL under the hood -> zero mallocs, zero frees!
                                    BLEAppUtil_invokeFunctionNoData(ProcessPasscode_invokeFromFWContext);
                                    break;
                                }
                            }
                        }
                        else // IMU_MODE_NORMAL_STREAM
                        {
                            ble_payload[7*i+0] = d.byte_16.accel_data[0];
                            ble_payload[7*i+1] = d.byte_16.accel_data[1];
                            ble_payload[7*i+2] = d.byte_16.accel_data[2];
                            ble_payload[7*i+3] = d.byte_16.gyro_data[0];
                            ble_payload[7*i+4] = d.byte_16.gyro_data[1];
                            ble_payload[7*i+5] = d.byte_16.gyro_data[2];
                            ble_payload[7*i+6] = d.byte_16.timestamp;
                        }
                    }
                }

                // Only send GATT notifications when in normal streaming mode
                if (current_imu_mode == IMU_MODE_NORMAL_STREAM &&
                    ble_payload[0] != INVALID_VALUE_FIFO)
                {
                    ble_payload[56] = event_marker;
                    event_marker = 0;
                    BLEAppUtil_invokeFunction(ble_helper, (char *)ble_payload);
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

    GPIO_setCallback(CONFIG_GPIO_INT1, INT1_callback_wakeup);
    GPIO_enableInt(CONFIG_GPIO_INT1);
}