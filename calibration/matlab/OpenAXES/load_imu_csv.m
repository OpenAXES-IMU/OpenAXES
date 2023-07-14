function [data] = load_imu_csv(filename, typeoffile)
   arguments
       filename string
       typeoffile string
   end

    switch typeoffile
        case 'csv_adxl355'
            opts = detectImportOptions(filename);
            opts.SelectedVariableNames = {'INDEX', 'ACCEL_ADXL355_0', 'ACCEL_ADXL355_1', 'ACCEL_ADXL355_2', 'GYRO_0', 'GYRO_1', 'GYRO_2' };
            data = readmatrix(filename, opts);

        case 'csv_bmi160'
            opts = detectImportOptions(filename);
            opts.SelectedVariableNames = {'INDEX', 'ACCEL_BMI160_0', 'ACCEL_BMI160_1', 'ACCEL_BMI160_2', 'GYRO_0', 'GYRO_1', 'GYRO_2' };
            data = readmatrix(filename, opts);

        otherwise
            data = [];
            disp('sorry, unknown inputtype')
            return
    end

end