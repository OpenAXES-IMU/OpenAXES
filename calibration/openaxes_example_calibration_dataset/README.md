# OpenAXES Example Calibration Dataset

**Since git repositories are not suitable for storing large amounts of static data, the dataset itself is kept in the research data repository of the LUH: https://doi.org/10.25835/oimsodoy. Download the dataset from there and extract it into the `/calibration` directory in this repository.**

This dataset contains calibration sequences of four OpenAXES IMUs, which were performed using the icosahedral calibration fixture described in the OpenAXES repository and paper.
For each IMU, five logs were recorded using the icosahedral calibration fixture according to the procedure described [below](#dataset-recording).
These can be found in the `logs` subdirectory.

Additionally, the `cuboid_logs` directory contains two logs for each IMU in which the device was only placed on the flat faces of the 3D-printed case.
As stated in the preprint, these are not useful for calibration and yield no usable calibration results.
They are only provided for completeness.

The `openaxes_example_calibration_dataset` subdirectory also contains the MATLAB scripts which were used to evaluate the dataset for [our preprint][3].
The script `coefficients-from-mat-files.py` can be used to extract the calibration coefficients from the `.mat` files.


## Evaluation

The `matlab_eval` subdirectory contains the scripts used to calculate the evaluation data presented in [our preprint][3].
To run it, you need the patched `imu_tk_matlab` code from the [OpenAXES repository](https://github.com/OpenAXES-IMU/OpenAXES) see the file [`/calibration/README.md`](https://github.com/OpenAXES-IMU/OpenAXES/blob/main/calibration/README.md) there.


## Dataset Recording

The example dataset was recorded for all four IMUs according to the procedure detailed below.

1. Make the environment as still as possible, for example:
   1. Choose a stury surface on which to perform the calibration, i.e. not a flimsy table that vibrates and sways with each touch.
   2. Avoid touching the surface on which you are working during the whole calibration procedure.
   3. Avoid walking around or moving on your chair during the whole calibration procedure.
   4. Remove all other people and other sources of movement from the room.
2. Insert the IMU into the 3D-printed case
3. Insert the case fully into the calibration fixture
4. Place the fixture on a flat surface in front of the operator who will perform the calibration movements
   A. Place the face with digit 1 facing upwards toward the ceiling
   B. Orient the digit 1 such that the operator can read it naturally (top side of the digit facing away from the operator).
5. Start recording the data with the following command:
   ```
   imu_control.py  imuX -b --still=3 --reset --skip=50 --mode=INDEX,ACCEL_ADXL355,ACCEL_BMI160,GYRO
   ```
   This command will start recording the raw accelerometer and gyroscope data from both inertial sensor chips on the IMU, along with an increasing 16 bit counter.
   * The argument `imuX` must be substituted with the designator of the IMU that is to be calibrated.
   * The flag `-b` will print battery state information to the console before the measurement starts.
   * The flag `--reset` will reset the internal state of the IMU before starting the measurement, specifically: *
     * Re-initialize the sensor ICs, e.g. performing gyroscope offset compensation
     * Reset the quaternion by calculating a new initial quaternion from the accelerometer data.
     * Reset the 16 bit INDEX counter
   * The argument `--still=3` emit a beeping sound when the IMU has lain still for 3 seconds after the last detected movement, prompting the operator to perform the next movement.
   * The argument `--skip=50` will discard the first 50 GATT notifications received, which corresponds to about half a second of data.
     This is necessary because the likelihood of packet loss is far greater at the beginning of the transmission due to unknown reasons.
6. Leave the IMU still on the surface for 40 seconds
7. Rotate the icosahedron such that the next numbered side faces up and is oriented correctly readable for the operator.
   * (_note:_ face number 2 contains the cutout for the IMU case and has no number)
   * It is helpful to draw a small arrow on each face, which points in the general direction of the next number, to speed up the time needed to find the correct face.
   * The movements should be practiced a few times to ensure smooth execution, because long periods of fumbling will increase the total calibration time, which might decrease the accuracy of the method.
8. Place the rotated icosahedron back on the flat surface and let it rest completely motionless until the beeping sound indicates that the static phase has been detected.
9. Again, make sure not to touch the table, walk around, or move on your chair during the still phase.
10. Repeat steps 7-9 for the next numbered face up until number 20.
11. After the first 20 faces, repeat steps 6-7 another 17-20 times, but rotate the icosahedron to a random face instead of the next numbered face.
12. After 37-40 faces, stop the recording by pressing `Ctrl+C` in the terminal running `imu_control.py`.

[1]: https://doi.org/10.1109/ICRA.2014.6907297 "Paper 'A robust and easy to implement method for IMU calibration without external equipments' by Tedaldi et al."
[2]: https://github.com/JzHuai0108/imu_tk_matlab "Matlab scripts of David Tedaldi's ICRA14 paper, a robust and easy to implement method for IMU calibration by Jianzhu Huai"
[3]: https://arxiv.org/pdf/2207.04801.pdf "Preprint 'Improved Calibration Procedure for Wireless Inertial
Measurement Units without Precision Equipment' by Webering et al."
[4]: https://bitbucket.org/alberto_pretto/imu_tk "The original imu_tk C++ implementation"