/*

@file  imu_calibration.h

@license  LGPL-3+

This file is part of the OpenAXES project, a wireless IMU.
Copyright 2023 Nils Stanislawski and Fritz Webering

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


#include "vectormath.h"

#ifndef APPLICATION_IMU_CALIBRATION_H_
#define APPLICATION_IMU_CALIBRATION_H_

typedef struct {
    Matrix3x3 accel_misalign_scale_adxl;
    Vector3 accel_bias_adxl;

    Matrix3x3 accel_misalign_scale_bmi;
    Vector3 accel_bias_bmi;

    Matrix3x3 gyro_misalign_scale;
    Vector3 gyro_bias;
} calibration_data_t;


/**
 * Apply calibration coefficients to a data sample.
 *
 * Can be gyroscope or accelerometer data, the formula is the same.
 * Only the correct misalign_scale matrix and bias vector need to be selected.
 */
Vector3 imu_calibration_apply(const Vector3 data, const Matrix3x3 misalign_scale, const Vector3 bias);

/**
 * Initialize the given data record to unit matrices and zero bias vectors.
 */
void imu_calibration_data_init(calibration_data_t *data);



#endif /* APPLICATION_IMU_CALIBRATION_H_ */
