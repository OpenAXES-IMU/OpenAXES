#!/usr/bin/env python3

import sys
import numpy as np
import scipy.io
import itertools
from scipy.spatial.transform.rotation import Rotation

num_orientations = 17

# for debugger
# sys.argv = ['', 'data_webering_paper1/calibration_results_imu9_adxl355_max_segments.mat', 'data_webering_paper1/calibration_results_imu9_bmi160_max_segments.mat']

if len(sys.argv) < 3:
    print(f'Usage: {sys.argv[0]} ADXL355.mat BMI160.mat')
    sys.exit(1)

adxl_data = scipy.io.loadmat(sys.argv[1])
bmi_data = scipy.io.loadmat(sys.argv[2])

adxl_bias_vector = np.mean(adxl_data['acc_bias_vectors'][:, :, num_orientations], 1)
adxl_scale_matrix = np.mean(adxl_data['acc_scale_matrices'][:, :, :, num_orientations], 2)
adxl_misal_matrix = np.mean(adxl_data['acc_misal_matrices'][:, :, :, num_orientations], 2)
adxl_misalign_scale = adxl_misal_matrix @ adxl_scale_matrix

bmi_bias_vector = np.mean(bmi_data['acc_bias_vectors'][:, :, num_orientations], 1)
bmi_scale_matrix = np.mean(bmi_data['acc_scale_matrices'][:, :, :, num_orientations], 2)
bmi_misal_matrix = np.mean(bmi_data['acc_misal_matrices'][:, :, :, num_orientations], 2)
bmi_misalign_scale = bmi_misal_matrix @ bmi_scale_matrix

adxl_gyro_bias_vector = np.array([0,0,0])
adxl_gyro_scale_matrix = np.mean(adxl_data['gyro_scale_matrices'][:, :, :, num_orientations], 2)
adxl_gyro_misal_matrix = np.mean(adxl_data['gyro_misal_matrices'][:, :, :, num_orientations], 2)
adxl_gyro_misalign_scale = adxl_gyro_misal_matrix @ adxl_gyro_scale_matrix

bmi_gyro_bias_vector = np.array([0,0,0])
bmi_gyro_scale_matrix = np.mean(bmi_data['gyro_scale_matrices'][:, :, :, num_orientations], 2)
bmi_gyro_misal_matrix = np.mean(bmi_data['gyro_misal_matrices'][:, :, :, num_orientations], 2)
bmi_gyro_misalign_scale = bmi_gyro_misal_matrix @ bmi_gyro_scale_matrix

all_matrices = [
    adxl_misalign_scale, adxl_bias_vector,
    bmi_misalign_scale, bmi_bias_vector,
    #adxl_gyro_misalign_scale, adxl_gyro_bias_vector,
    bmi_gyro_misalign_scale, bmi_gyro_bias_vector
]

print('\nMismatch between ADXL and BMI calibrated gyro scale matrices is:')
print(adxl_gyro_scale_matrix @ np.linalg.inv(bmi_gyro_scale_matrix))

print('\nMismatch between ADXL and BMI calibrated gyro misalignment matrices is:')
misal_mismatch = adxl_gyro_misal_matrix @ np.linalg.inv(bmi_gyro_misal_matrix)
print(misal_mismatch)

print('\nQuotient of ADXL and BMI gyro misalign_scale matrices is:')
misal_quotient = adxl_gyro_misal_matrix / bmi_gyro_misal_matrix
print(misal_quotient)

print('\nMismatch between ADXL and BMI calibrated gyro misalignment as Euler angles:')
r = Rotation.from_matrix(misal_mismatch)
print(np.rad2deg(r.as_euler('xyz')))

print('\nAll matrices:')
for m in all_matrices:
    print(m)

all_coefficients = itertools.chain(
    *(x.flatten().tolist() for x in all_matrices)
)

print('\nAll coefficients:')
print(', '.join(map(str, all_coefficients)))
print()
