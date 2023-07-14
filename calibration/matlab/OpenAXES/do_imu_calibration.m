% This file demostrates how to calculate the calibration coefficients
% for an OpenAXES IMU from a csv file recorded according to the
% calibration recommended procedure.

%% Cleanup and setup
clearvars

% Add calibration source files to the search path
path('matlab/OpenAXES', path)
path('matlab/imu_tk_matlab/srcs', path)

filename = 'log_imu9_2022-06-22_22-13-55.csv';
% Try to determine the IMU name from the filename by 
[token, remainder] = strtok(filename, '_');
[imu_name, remainder] = strtok(remainder, '_');

%% Calculate calibration based on both accelerometers

data_adxl = load_imu_csv(filename, 'csv_adxl355');
assert(~isempty(data_adxl))

disp('----------------------')
disp('Calculating ADXL-accelerometer-based calibration coefficients')
[adxl_acc_misal_matrix, adxl_acc_scale_matrix, adxl_acc_bias_vector, ...
    adxl_gyro_misal_matrix, adxl_gyro_scale_matrix, adxl_gyro_bias_vector, ...
    adxl_num_segments] = calibrate(data_adxl);

disp('----------------------')
disp('Calculating BMI-accelerometer-based calibration coefficients')
data_bmi = load_imu_csv(filename, 'csv_bmi160');
assert(~isempty(data_bmi))

[bmi_acc_misal_matrix, bmi_acc_scale_matrix, bmi_acc_bias_vector, ...
    bmi_gyro_misal_matrix, bmi_gyro_scale_matrix, bmi_gyro_bias_vector, ...
    bmi_num_segments] = calibrate(data_bmi);

%% Show raw calibration results for both accelerometers side by side

% disp('[adxl_acc_misal_matrix  bmi_acc_misal_matrix]:')
% disp([adxl_acc_misal_matrix  bmi_acc_misal_matrix])
% disp('[adxl_acc_scale_matrix  bmi_acc_scale_matrix]:')
% disp([adxl_acc_scale_matrix  bmi_acc_scale_matrix])
% disp('[adxl_acc_bias_vector  bmi_acc_bias_vector]:')
% disp([adxl_acc_bias_vector  bmi_acc_bias_vector])
% disp('[adxl_gyro_misal_matrix  bmi_gyro_misal_matrix]:')
% disp([adxl_gyro_misal_matrix  bmi_gyro_misal_matrix])
% disp('[adxl_gyro_scale_matrix  bmi_gyro_scale_matrix]:')
% disp([adxl_gyro_scale_matrix  bmi_gyro_scale_matrix])
% disp('[adxl_gyro_bias_vector  bmi_gyro_bias_vector]:')
% disp([adxl_gyro_bias_vector  bmi_gyro_bias_vector])

%% Calculate calibration coefficients for the OpenAXES firmware

% First combine the misalignment and the scaling matrices for each sensor
adxl_acc_misalign_scale = adxl_acc_misal_matrix * adxl_acc_scale_matrix;
bmi_acc_misalign_scale = bmi_acc_misal_matrix * bmi_acc_scale_matrix;
adxl_gyro_misalign_scale = adxl_gyro_misal_matrix * adxl_gyro_scale_matrix;
bmi_gyro_misalign_scale = bmi_gyro_misal_matrix * bmi_gyro_scale_matrix;

% The IMU needs only one matrix for each sensor, but we have two for the
% gyro: one based on the ADXL accel data and one based on the BMI accel
% data. We use the gyro coefficients based on the BMI sensor here, because
% the ADXL sensor is supposed to be optional. The adxl_gyro_misalign_scale
% and adxl_gyro_bias_vector could be used alternatively.
all_coefficients = [
    adxl_acc_misalign_scale(:); adxl_acc_bias_vector(:);
    bmi_acc_misalign_scale(:); bmi_acc_bias_vector(:);
    bmi_gyro_misalign_scale(:); bmi_gyro_bias_vector(:);
]';

% Convert the 36 coefficients to a whitespace separated string
str_coefficients = strjoin(string(all_coefficients), ' ');

disp('----------------------')
disp('Use the following command to write the coefficients to the IMU:')
disp(append('imu_control.py ', imu_name, ' --write-calibration-full="', str_coefficients, '"'))
