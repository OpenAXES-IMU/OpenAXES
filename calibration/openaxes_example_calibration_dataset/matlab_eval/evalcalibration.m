clear all;
close all;
clc;

% Add calibration source files to the search path
path('matlab/OpenAXES', path)
path('matlab/imu_tk_matlab/srcs', path)

%% build mean values of multiple calibration sequences

num_of_sets = 5;
lognum = 1151; 
savedfile = 'calib_results_mean_bmi_115X.mat'; %name of file for mean calib results

%load datasets
accelparam = zeros(9,num_of_sets);
gyroparam = zeros(12,num_of_sets);
for i=lognum:(lognum+(num_of_sets-1))
    filename = strcat( 'calib_results_' , string(i) , '.mat');
    load(filename);
    accelparam(:,i-(lognum-1))=theta_pr_opt;
    gyroparam(:,i-(lognum-1))=[theta_pr_gyro, estimate_bias_x, estimate_bias_y, estimate_bias_z];
end

%build means
theta_pr_opt = [mean(accelparam(1,:)),mean(accelparam(2,:)),mean(accelparam(3,:)),mean(accelparam(4,:)),mean(accelparam(5,:)),mean(accelparam(6,:)),mean(accelparam(7,:)),mean(accelparam(8,:)),mean(accelparam(9,:))];
theta_pr_gyro = [mean(gyroparam(1,:)),mean(gyroparam(2,:)),mean(gyroparam(3,:)),mean(gyroparam(4,:)),mean(gyroparam(5,:)),mean(gyroparam(6,:)),mean(gyroparam(7,:)),mean(gyroparam(8,:)),mean(gyroparam(9,:))];
estimate_bias_x = mean(gyroparam(10,:));
estimate_bias_y = mean(gyroparam(11,:));
estimate_bias_z = mean(gyroparam(12,:));

%save as .mat file
save(savedfile, 'theta_pr_gyro', 'theta_pr_opt', 'estimate_bias_x', 'estimate_bias_y', 'estimate_bias_z')


%% calibrate datafile with calib results in matrices 

%
datafile = ('calib_data_1152.csv'); %calib sequence to correct 
data = readmatrix(datafile);
matrices = {'calib_results_1151.mat','calib_results_1152.mat', 'calib_results_1153.mat', 'calib_results_1154.mat', 'calib_results_1155.mat', 'calib_results_mean_bmi_115X.mat'}; %calib results used to correct sequence


num_mats = length(matrices);
results = zeros(21, num_mats);


for j = 1:num_mats

    %load calib result
    mat = load(char(matrices(j)));
    %print(matrices(j))
    misal_accel = [1, -mat.theta_pr_opt(1), mat.theta_pr_opt(2); 0, 1, -mat.theta_pr_opt(3); 0, 0, 1];
    scale_accel = diag([mat.theta_pr_opt(4), mat.theta_pr_opt(5), mat.theta_pr_opt(6)]);
    bias_accel = [mat.theta_pr_opt(7), mat.theta_pr_opt(8), mat.theta_pr_opt(9)];

    misal_gyro = [1, mat.theta_pr_gyro(2), mat.theta_pr_gyro(3); mat.theta_pr_gyro(4), 1, mat.theta_pr_gyro(6); mat.theta_pr_gyro(7), mat.theta_pr_gyro(8), 1];
    scale_gyro = [mat.theta_pr_gyro(1), 0, 0; 0, mat.theta_pr_gyro(5), 0; 0, 0, mat.theta_pr_gyro(9)];
    bias_gyro = [mat.estimate_bias_x, mat.estimate_bias_y, mat.estimate_bias_z];

    corr_accel = zeros(length(data),3);
    corr_gyro = zeros(length(data),3);

    %correct data set
    for i = 1:length(data)
        corr_accel(i,:) = (misal_accel * (scale_accel * (data(i,2:4) - bias_accel)'))';
        corr_gyro(i,:) = (misal_gyro * (scale_gyro * (data(i,5:7) - bias_gyro)'))';
    end 
    corr_data = [data(:,1) corr_accel corr_gyro ];
    
    results(:,j) = calibrate(corr_data, 'interpolate', zeros(9,1), zeros(9,1));
end