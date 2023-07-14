% Cleanup
clearvars;
close all;
clc;

% Add calibration source files to the search path
path('matlab/OpenAXES', path)
path('matlab/imu_tk_matlab/srcs', path)

%% Settings and data import

imus = { 'imu1', 'imu2', 'imu6', 'imu9' };
accelerometers = { 'adxl355', 'bmi160' };
%imus = {  'imu6' };
%accelerometers = { 'bmi160' };

for j = 1:length(imus)
    for k = 1:length(accelerometers)
        imu_name = imus{j};
        accelerometer = accelerometers{k};
        case_name = [imu_name '_' accelerometer ];
        
        % Load data files
        dir_list = dir(['../logs/log_' imu_name '*.csv']);
        data_arrays = cell(length(dir_list), 1);
        for i = 1:length(dir_list)
            filename = [dir_list(i).folder '/' dir_list(i).name];
            filetype = ['csv_' accelerometer];
            data_arrays{i} = load_imu_csv(filename, filetype);
        end
        
        run_calibration_for_imu(data_arrays, case_name);
    end
end

%% Run calibration for cuboid calibration sequence

imus = { 'imu1', 'imu2', 'imu6', 'imu9' };
accelerometers = { 'adxl355', 'bmi160' };
%imus = {  'imu6' };
%accelerometers = { 'bmi160' };

for j = 1:length(imus)
    for k = 1:length(accelerometers)
        imu_name = imus{j};
        accelerometer = accelerometers{k};
        case_name = [ 'cuboid_' imu_name '_' accelerometer ];
        
        % Load data files
        dir_list = dir(['../cuboid_logs/log_' imu_name '*.csv']);
        data_arrays = cell(length(dir_list), 1);
        for i = 1:length(dir_list)
            filename = [dir_list(i).folder '/' dir_list(i).name];
            filetype = ['csv_' accelerometer];
            data_arrays{i} = load_imu_csv(filename, filetype);
        end
        
        run_calibration_for_imu(data_arrays, case_name);
    end
end

%% Run evaluation for Tedaldi's xsens IMU

xsens_acc = readmatrix('../xsens_acc.mat', FileType='text');
xsens_acc(:,2:4) = xsens_acc(:,2:4) - 32678;
xsens_gyro = readmatrix('../xsens_gyro.mat', FileType='text');
xsens_gyro(:, 2:4) = xsens_gyro(:, 2:4) / 6258.0;
data = [xsens_acc(:,1:4) xsens_gyro(:, 2:4)];
data_arrays = { data };
run_calibration_for_imu(data_arrays, 'xsens');


%% Compare gyro misalignment angles to reference

best_gyro_misal_matrices = zeros(3,3,N);
for i = 1:N
    best_gyro_misal_matrices(:, :, i) = gyro_misal_matrices(:, :, i, max_segments_list(i));
end
ref_gyro_misal_matrix = mean(best_gyro_misal_matrices, 3);
ref_gyro_misal_angles = misalignment_angles(ref_gyro_misal_matrix);

gyro_angle_differences = nan(6, N, max(max_segments_list));
%gyro_mse_angle_errors_deg = zeros(N, max(max_segments_list));
gyro_mean_angle_errors_deg = zeros(N, max(max_segments_list));
%gyro_std_angle_errors_deg = zeros(N, max(max_segments_list));
for i = 1:N
    num_seg = max_segments_list(i);
    gyro_angle_differences(:, i, 1:num_seg) = misalignment_angles(gyro_misal_matrices(:, :, i, 1:num_seg)) - ref_gyro_misal_angles;
    %gyro_mse_angle_errors_deg(i, :) = rad2deg(sqrt(sum(gyro_angle_differences(:, i, :).^2, 1)));
    gyro_mean_angle_errors_deg(i, :) = rad2deg(mean(abs(gyro_angle_differences(:, i, :)), 1));
    %gyro_std_angle_errors_deg(i, :) = rad2deg(std(gyro_angle_differences(:, i, :), 1));
end

figure
title('Gyro axis misalignment estimation error over N')
xlabel('Number of orientations N')
ylabel('Misalignment angle (degrees)')
hold on
gyro_mean_angle_errors_deg(gyro_mean_angle_errors_deg > 0.3) = nan;
for i = 1:N
    plot(gyro_mean_angle_errors_deg(i,:), 'DisplayName', ['Sequence ' num2str(i)])
end
ref_mean = rad2deg(mean(abs(ref_gyro_misal_angles)));
yline(ref_mean, 'red', 'DisplayName', 'Reference axis misalignment')
legend('Location', 'east');
saveas(gcf, 'gyro_misalignment.png', 'png')
%ylim([0 ref_mean])

%% Compare accel misalignment angles to reference
best_acc_misal_matrices = zeros(3,3,N);
for i = 1:N
    best_acc_misal_matrices(:, :, i) = acc_misal_matrices(:, :, i, max_segments_list(i));
end
ref_acc_misal_matrix = mean(best_acc_misal_matrices, 3);
ref_acc_misal_angles = misalignment_angles(ref_acc_misal_matrix);

acc_angle_differences = nan(6, N, max(max_segments_list));
acc_mean_angle_errors_deg = zeros(N, max(max_segments_list));
for i = 1:N
    num_seg = max_segments_list(i);
    acc_angle_differences(:, i, 1:num_seg) = misalignment_angles(acc_misal_matrices(:, :, i, 1:num_seg)) - ref_acc_misal_angles;
    acc_mean_angle_errors_deg(i, :) = rad2deg(mean(abs(acc_angle_differences([1 2 4], i, :)), 1));
end

figure
title('Accel axis misalignment estimation error over N')
xlabel('Number of orientations N')
ylabel('Misalignment angle (degrees)')
hold on
acc_mean_angle_errors_deg(acc_mean_angle_errors_deg >0.5) = nan;
for i = 1:N
    plot(acc_mean_angle_errors_deg(i,:), 'DisplayName', ['Sequence ' num2str(i)])
end
ref_mean = rad2deg(mean(abs(ref_acc_misal_angles)));
yline(ref_mean, 'red', 'DisplayName', 'Reference axis misalignment')
legend('Location', 'east');
saveas(gcf, 'accel_misalignment.png', 'png')
%ylim([0 inf])

%% Compare gyro scaling factors
best_gyro_scale_factors = zeros(3,N);
for i = 1:N
    best_gyro_scale_factors(1, i) = gyro_scale_matrices(1, 1, i, max_segments_list(i));
    best_gyro_scale_factors(2, i) = gyro_scale_matrices(2, 2, i, max_segments_list(i));
    best_gyro_scale_factors(3, i) = gyro_scale_matrices(3, 3, i, max_segments_list(i));
end
ref_gyro_scale_factors = mean(best_gyro_scale_factors, 2);

gyro_scale_differences = nan(3, N, max(max_segments_list));
gyro_scale_mean_errors = zeros(N, max(max_segments_list));
for i = 1:N
    num_seg = max_segments_list(i);
    gyro_scale_differences(1, i, 1:num_seg) = gyro_scale_matrices(1, 1, i, 1:num_seg) - ref_gyro_scale_factors(1);
    gyro_scale_differences(2, i, 1:num_seg) = gyro_scale_matrices(2, 2, i, 1:num_seg) - ref_gyro_scale_factors(2);
    gyro_scale_differences(3, i, 1:num_seg) = gyro_scale_matrices(3, 3, i, 1:num_seg) - ref_gyro_scale_factors(3);
    gyro_scale_mean_errors(i, :) = mean(abs(gyro_scale_differences(:, i, :)), 1);
end

figure
title('Gyro scale factor estimation error over N')
xlabel('Number of orientations N')
ylabel('Mean absolute scale factor difference')
hold on
gyro_scale_mean_errors(gyro_scale_mean_errors > 0.2) = nan;
for i = 1:N
    plot(gyro_scale_mean_errors(i,:), 'DisplayName', ['Sequence ' num2str(i)])
end
%ylim([0 inf])
legend('Location', 'east');
saveas(gcf, 'gyro_scale.png', 'png')


%% Compare accel scaling factors
best_acc_scale_factors = zeros(3,N);
for i = 1:N
    best_acc_scale_factors(1, i) = acc_scale_matrices(1, 1, i, max_segments_list(i));
    best_acc_scale_factors(2, i) = acc_scale_matrices(2, 2, i, max_segments_list(i));
    best_acc_scale_factors(3, i) = acc_scale_matrices(3, 3, i, max_segments_list(i));
end
ref_acc_scale_factors = mean(best_acc_scale_factors, 2);

acc_scale_differences = nan(3, N, max(max_segments_list));
acc_scale_mean_errors = zeros(N, max(max_segments_list));
for i = 1:N
    num_seg = max_segments_list(i);
    acc_scale_differences(1, i, 1:num_seg) = acc_scale_matrices(1, 1, i, 1:num_seg) - ref_acc_scale_factors(1);
    acc_scale_differences(2, i, 1:num_seg) = acc_scale_matrices(2, 2, i, 1:num_seg) - ref_acc_scale_factors(2);
    acc_scale_differences(3, i, 1:num_seg) = acc_scale_matrices(3, 3, i, 1:num_seg) - ref_acc_scale_factors(3);
    acc_scale_mean_errors(i, :) = mean(abs(acc_scale_differences(:, i, :)), 1);
end

figure
title('Accel scale factor estimation error over N')
xlabel('Number of orientations N')
ylabel('Mean absolute scale factor difference')
hold on
acc_scale_mean_errors(acc_scale_mean_errors > 0.2) = nan;
for i = 1:N
    plot(acc_scale_mean_errors(i,:), 'DisplayName', ['Sequence ' num2str(i)])
end
%ylim([0 inf])
legend('Location', 'east');
saveas(gcf, 'accel_scale.png', 'png')

%% Compare accel bias values
best_acc_bias_values = zeros(3,N);
for i = 1:N
    best_acc_bias_values(:, i) = acc_bias_vectors(:, i, max_segments_list(i));
end
ref_acc_bias_values = mean(best_acc_bias_values, 2);

acc_bias_differences = nan(3, N, max(max_segments_list));
acc_bias_mean_errors = zeros(N, max(max_segments_list));
for i = 1:N
    num_seg = max_segments_list(i);
    acc_bias_differences(:, i, 1:num_seg) = acc_bias_vectors(:, i, 1:num_seg) - ref_acc_bias_values;
    acc_bias_mean_errors(i, :) = mean(abs(acc_bias_differences(:, i, :)), 1);
end

figure
hold on
acc_bias_mean_errors(acc_bias_mean_errors > 1) = nan;
plot(acc_bias_mean_errors')
legend
ylim([0 inf])


%%
function [] = run_calibration_for_imu(data_arrays, case_name)
N = length(data_arrays);
max_max_segments = 10000;
acc_bias_vectors = zeros(3, N, 1);
acc_scale_matrices = zeros(3, 3, N, 1);
acc_misal_matrices = zeros(3, 3, N, 1);
gyro_bias_vectors = zeros(3, N, 1);
gyro_scale_matrices = zeros(3, 3, N, 1);
gyro_misal_matrices = zeros(3, 3, N, 1);
max_segments_list = zeros(N, 1);

for i = 1:N
    data = data_arrays{i};

    disp(['====== Calibrating ' case_name ' sequence ' num2str(i) '/' num2str(N) ' ======'])
    max_segments = max_max_segments;
    static_threshold = 0;
    while max_segments > 0
        disp(['====== Calibration for ' num2str(max_segments) ' segments ======'])
        [acc_misal_matrix, acc_scale_matrix, acc_bias_vector, ...
            gyro_misal_matrix, gyro_scale_matrix, gyro_bias_vector, ...
            num_segments, static_threshold] ...
            = calibrate(data, 'interpolate', max_segments, static_threshold);
        if max_segments == max_max_segments
            max_segments_list(i) = num_segments;
            max_segments = num_segments;
        end
        disp(['Found ' num2str(num_segments) ' segments. Gyro and acc misalignment angles are:'])
        disp([ misalignment_angles(gyro_misal_matrix, 'deg'); ...
            misalignment_angles(acc_misal_matrix, 'deg')])
        
        acc_bias_vectors(:, i, num_segments) = acc_bias_vector;
        acc_scale_matrices(:, :, i, num_segments) = acc_scale_matrix;
        acc_misal_matrices(:, :, i, num_segments) = acc_misal_matrix;
        gyro_bias_vectors(:, i, num_segments) = gyro_bias_vector;
        gyro_scale_matrices(:, :, i, num_segments) = gyro_scale_matrix;
        gyro_misal_matrices(:, :, i, num_segments) = gyro_misal_matrix;

        max_segments = max_segments - 1;
    end
    
end
results_file = ['calibration_results_' case_name '_max_segments'];
save(results_file, 'N', 'acc_bias_vectors', 'acc_scale_matrices', 'acc_misal_matrices', ...
    'gyro_scale_matrices', 'gyro_misal_matrices', 'max_segments_list');
end