#ifndef SENSORFUSION_H
#define SENSORFUSION_H

#include "Fusion.h"

// Global AHRS instance
extern FusionAhrs ahrs;

typedef struct data_sample
{
    FusionQuaternion quaternion;
    FusionVector linear_accel;
    int16_t       raw_accel[3];
	int16_t       raw_gyro[3];
	uint16_t      timestamp;
} data_sample_t;

void Init_Fusion_AHRS(void);

int SensorFusion(data_sample_t *data);

#endif