#include "Fusion.h"
#include <stdint.h>

#include "sensorfusion.h"


// Pre-calculate FP division
const float GYRO_SENSITIVITY_INV = 2000.0f / 32768.0f; 
const float ACCEL_SENSITIVITY_INV = 16.0f / 32768.0f; 
const float MICROSECONDS_TO_SECONDS = 1.0f / 1000000.0f;

// Global AHRS instance
FusionAhrs ahrs;

void Init_Fusion_AHRS(void) {
    FusionAhrsInitialise(&ahrs);

    FusionAhrsSettings settings = {
        // Starting point parameters, may need to tune these or maybe good enough
        .convention = FusionConventionNwu,
        .gain = 0.5f,
        .gyroscopeRange = 2000.0f,
        .accelerationRejection = 10.0f,
        .recoveryTriggerPeriod = 5 * 100
    };
    FusionAhrsSetSettings(&ahrs, &settings);
}



int SensorFusion(data_sample_t *data)
{
    static uint16_t previous_timestamp = 0;
    static bool first_run = true;

    // Discard the first sample and get timestamp
    if (first_run)
    {
        previous_timestamp = data->timestamp;
        first_run = false;
        // Skip the AHRS update for the first sample to avoid a massive dt
        return -1; 
    }

    // Calculate Delta Time from timestamps
    uint16_t delta_ticks = data->timestamp - previous_timestamp;
    previous_timestamp = data->timestamp;
    float deltaTime = (float)delta_ticks * MICROSECONDS_TO_SECONDS;

    // int to float
    FusionVector gyroscope = {
        .axis.x = data->raw_gyro[0] * GYRO_SENSITIVITY_INV,
        .axis.y = data->raw_gyro[1] * GYRO_SENSITIVITY_INV,
        .axis.z = data->raw_gyro[2] * GYRO_SENSITIVITY_INV,
    };

    FusionVector accelerometer = {
        .axis.x = data->raw_accel[0] * ACCEL_SENSITIVITY_INV,
        .axis.y = data->raw_accel[1] * ACCEL_SENSITIVITY_INV,
        .axis.z = data->raw_accel[2] * ACCEL_SENSITIVITY_INV,
    };

    
    FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

    data->quaternion = FusionAhrsGetQuaternion(&ahrs);
    data->linear_accel = FusionAhrsGetLinearAcceleration(&ahrs);

    return 0;
}
