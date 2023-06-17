// Copyright 2010 Sebastian Madgwick
// Permission for use in this project granted by S. Madgwick on 2023-06-18
// Please do not use this, instead use the new version called "Fusion": https://github.com/xioTechnologies/Fusion

/* Madgwick Filter Prototypes */

// System constants
#define deltat 0.01f // sampling period in seconds (shown as 10 ms)
#define gyroMeasError 3.14159265358979f * (0.12f / 180.0f) // gyroscope measurement error in rad/s (shown as 120 mdeg/s)
#define gyroMeasDrift 3.14159265358979f * (0.006f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.006 deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta


void madgwickImuFilterUpdate(
	float w_x, float w_y, float w_z,
	float a_x, float a_y, float a_z,
	float *q_1, float *q_2, float *q_3, float *q_4);

void madgwickMargFilterUpdate(
	float w_x, float w_y, float w_z,
	float a_x, float a_y, float a_z,
	float m_x, float m_y, float m_z,
	float *q_1, float *q_2, float *q_3, float *q_4,
	float *b_x_e, float *b_z_e,
	float *w_bx_e, float *w_by_e, float *w_bz_e);
