% Cleanup
clearvars;
close all;
clc;

% Add calibration source files to the search path
path('matlab/OpenAXES', path)
path('matlab/imu_tk_matlab/srcs', path)

%% Settings and data import

imu_name = 'imu2';
filetype = 'csv_adxl355';
calibration_runs = 2; % The first N runs are used for calibration, the rest are used to verify the calibration

% Load data files
dir_list = dir(['openaxes_example_calibration_dataset/logs/log_' imu_name '*.csv']);
datafiles = cell(length(dir_list), 1);
for i = 1:length(dir_list)
    datafiles{i} = [dir_list(i).folder '/' dir_list(i).name];
end

%% Calculate Calibration for all files
N = length(dir_list);
acc_bias_vectors = zeros(1, 3, N);
acc_scale_matrices = zeros(3, 3, N);
acc_misal_matrices = zeros(3, 3, N);
gyro_bias_vectors = zeros(1, 3, N);
gyro_scale_matrices = zeros(3, 3, N);
gyro_misal_matrices = zeros(3, 3, N);
num_segments_list = zeros(N, 1);

for i = 1:length(datafiles)
    filename = datafiles{i};
    
    [acc_misal_matrix, acc_scale_matrix, acc_bias_vector, ...
        gyro_misal_matrix, gyro_scale_matrix, gyro_bias_vector, ...
        num_segments] = calibrate(filename, filetype);
    
%     disp('Accel Bias Vector')
%     disp(acc_bias_vector); % accelerator bias vector
%     disp('Accel Scaling Matrix')
%     disp(acc_scale_matrix); % acc
%     disp('Accel Misalignment Matrix')
%     disp(acc_misal_matrix); % acc
%     disp('Gyro Bias Vector')
%     disp(gyro_bias_vector); % gyro
%     disp('Gyro Scaling Matrix')
%     disp(gyro_scale_matrix); % gyro
%     disp('Gyro Misalignment Matrix')
%     disp(gyro_misal_matrix); % gyro

    num_segments_list(i) = num_segments;
    acc_bias_vectors(:, :, i) = acc_bias_vector;
    acc_scale_matrices(:, :, i) = acc_scale_matrix;
    acc_misal_matrices(:, :, i) = acc_misal_matrix;
    gyro_bias_vectors(:, :, i) = gyro_bias_vector;
    gyro_scale_matrices(:, :, i) = gyro_scale_matrix;
    gyro_misal_matrices(:, :, i) = gyro_misal_matrix;
end

save('calibration_results')

%%
disp('std(num_segments_list) |  mean(num_segments_list)')
disp([std(num_segments_list) mean(num_segments_list)])
disp('std(acc_bias_vectors) |  mean(acc_bias_vectors)')
disp([std(acc_bias_vectors, 0, 3) mean(acc_bias_vectors, 3)])
disp('std(acc_scale_matrices) |  mean(acc_scale_matrices)')
disp([std(acc_scale_matrices, 0, 3) mean(acc_scale_matrices, 3)])
disp('std(acc_misal_matrices) |  mean(acc_misal_matrices)')
disp([std(acc_misal_matrices, 0, 3) mean(acc_misal_matrices, 3)])
disp('std(gyro_bias_vectors) |  mean(gyro_bias_vectors)')
disp([std(gyro_bias_vectors, 0, 3) mean(gyro_bias_vectors, 3)])
disp('std(gyro_scale_matrices) |  mean(gyro_scale_matrices)')
disp([std(gyro_scale_matrices, 0, 3) mean(gyro_scale_matrices, 3)])
disp('std(gyro_misal_matrices) |  mean(gyro_misal_matrices)')
disp([std(gyro_misal_matrices, 0, 3) mean(gyro_misal_matrices, 3)])

%% Calculate calibrated data and try to calibrate again

% use the first cal_N data frames as calibration correction
cal_N = calibration_runs;
bias_accel = mean(acc_bias_vectors(:,:,1:cal_N), 3);
scale_accel = mean(acc_scale_matrices(:,:,1:cal_N), 3);
misal_accel = mean(acc_misal_matrices(:,:,1:cal_N), 3);
bias_gyro = mean(gyro_bias_vectors(:,:,1:cal_N), 3);
scale_gyro = mean(gyro_scale_matrices(:,:,1:cal_N), 3);
misal_gyro = mean(gyro_misal_matrices(:,:,1:cal_N), 3);

corr_acc_bias_vectors = zeros(1, 3, N-cal_N);
corr_acc_scale_matrices = zeros(3, 3, N-cal_N);
corr_acc_misal_matrices = zeros(3, 3, N-cal_N);
corr_gyro_bias_vectors = zeros(1, 3, N-cal_N);
corr_gyro_scale_matrices = zeros(3, 3, N-cal_N);
corr_gyro_misal_matrices = zeros(3, 3, N-cal_N);
corr_num_segments_list = zeros(N, 1-cal_N);

for i = 1:length(datafiles)-cal_N % use the remaining data files for verification
    filename = datafiles{i+cal_N};

    data = load_imu_csv(filename, filetype);
    corr_accel = zeros(length(data),3);
    corr_gyro = zeros(length(data),3);

    %correct data set
    for k = 1:length(data)
        corr_accel(k,:) = (misal_accel * (scale_accel * (data(k,2:4) - bias_accel)'))';
        corr_gyro(k,:) = (misal_gyro * (scale_gyro * (data(k,5:7) - bias_gyro)'))';
    end 
    corr_data = [data(:,1) corr_accel corr_gyro ];
    
    [acc_misal_matrix, acc_scale_matrix, acc_bias_vector, ...
        gyro_misal_matrix, gyro_scale_matrix, gyro_bias_vector, ...
        num_segments] = calibrate(corr_data, 'interpolate');
    
    corr_num_segments_list(i) = num_segments;
    corr_acc_bias_vectors(:, :, i) = acc_bias_vector;
    corr_acc_scale_matrices(:, :, i) = acc_scale_matrix;
    corr_acc_misal_matrices(:, :, i) = acc_misal_matrix;
    corr_gyro_bias_vectors(:, :, i) = gyro_bias_vector;
    corr_gyro_scale_matrices(:, :, i) = gyro_scale_matrix;
    corr_gyro_misal_matrices(:, :, i) = gyro_misal_matrix;
end

%% Show re-calibration results

disp('std(corr_num_segments_list) |  mean(corr_num_segments_list)')
disp([std(corr_num_segments_list) mean(corr_num_segments_list)])
disp('std(corr_acc_bias_vectors) |  mean(corr_acc_bias_vectors)')
disp([std(corr_acc_bias_vectors, 0, 3) mean(corr_acc_bias_vectors, 3)])
disp('std(corr_acc_scale_matrices) |  mean(corr_acc_scale_matrices)')
disp([std(corr_acc_scale_matrices, 0, 3) mean(corr_acc_scale_matrices, 3)])
disp('std(corr_acc_misal_matrices) |  mean(corr_acc_misal_matrices)')
disp([std(corr_acc_misal_matrices, 0, 3) mean(corr_acc_misal_matrices, 3)])
disp('std(corr_gyro_bias_vectors) |  mean(corr_gyro_bias_vectors)')
disp([std(corr_gyro_bias_vectors, 0, 3) mean(corr_gyro_bias_vectors, 3)])
disp('std(corr_gyro_scale_matrices) |  mean(corr_gyro_scale_matrices)')
disp([std(corr_gyro_scale_matrices, 0, 3) mean(corr_gyro_scale_matrices, 3)])
disp('std(corr_gyro_misal_matrices) |  mean(corr_gyro_misal_matrices)')
disp([std(corr_gyro_misal_matrices, 0, 3) mean(corr_gyro_misal_matrices, 3)])
