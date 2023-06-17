/*

@file  imu_calibration.c

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


#include "imu_calibration.h"

Vector3 imu_calibration_apply(const Vector3 data, const Matrix3x3 misalign_scale, const Vector3 bias)
{
    Vector3 unbiased = Vector3_add(data, bias);
    return Matrix3x3_timesColumnVector(misalign_scale, unbiased);
}

void imu_calibration_data_init(calibration_data_t *data)
{
    data->accel_misalign_scale_adxl = (Matrix3x3) {{
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    }};
    data->accel_misalign_scale_bmi = (Matrix3x3) {{
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    }};
    data->gyro_misalign_scale = (Matrix3x3) {{
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    }};
    data->accel_bias_adxl = (Vector3) {{ 0, 0, 0 }};
    data->accel_bias_bmi = (Vector3) {{ 0, 0, 0 }};
    data->gyro_bias = (Vector3) {{ 0, 0, 0 }};
}

