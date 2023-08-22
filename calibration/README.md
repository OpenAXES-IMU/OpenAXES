# OpenAXES Calibration Tools and Example Dataset

The `calibration` subdirectory contains the scripts used to calculate the calibration coefficients for OpenAXES IMUs (or any other suitable IMU) using the method published by Tedaldi et al. in their 2014 paper [A robust and easy to implement method for IMU calibration without external equipments][1]

For calibrating our own IMUs we [used the MATLAB implementation by Jianzhu Huai][2] (state as of 2017).
The `imu_tk_matlab` code is included in this repository as a submodule only, because it has no license that would permit us to include a copy of the code itself.
We made some changes, improvements, and bugfixes to the original code, which are contained in the [patches](/calibration/matlab/patches/) subdirectory.
The changes are described in the `CHANGES.md` inside the patch, as well as in greater detail in [our preprint on arxiv.org][3].

Alternatively, one could try to use [the original C++ implementation by Tedaldi et al.][4], which is released under the BSD license.
We used the MATLAB version because it was easier to get started.
We did not test whether it also works with [Octave](https://octave.org/), a free implementation of the MATLAB language.


## Example Calibration Dataset

The subdirectory `/calibration/openaxes_example_calibration_dataset` contains example calibration recordings from four OpenAXES IMUs, which can be used to explore and understand the calibration algorithm.
Please refer to the [separate README file for the dataset](/calibration/openaxes_example_calibration_dataset/README.md) for more information.

**Since git repositories are not suitable for storing large amounts of static data, the dataset itself is kept in the research data repository of the LUH: https://doi.org/10.25835/oimsodoy. Download the dataset from there and extract it into the `/calibration` directory in this repository.**


## Recording Calibration Data

We recommend using the icosahedral calibration fixture for reliable and repeatable recording of calibration data for the OpenAXES IMU.
The 3D-printable model can be found in [the /hardware/calibrator/ directory](/hardware/calibrator/D20-Calibrator.3mf) in 3MF format for PrusaSlicer.

Detailed step-by-step instructions for recording a calibration sequence can be found in [the dataset README file](/calibration/openaxes_example_calibration_dataset/README.md#dataset-recording).


## Calculating Calibration Coefficients

Once calibration recordings have been performed, the `imu_tk` algorithm can be used to calculate the corrective calibration coefficients for that IMU.
We recommend doing a few practice runs before calibrating units for production use.
It is possible and recommended to perform multiple calibration recordings for each IMU and compare the results to make sure they are consistent.
Otherwise, small mistakes in the procedure in one calibration run could result in suboptimal calibration.

To use the MATLAB implementation of the `imu_tk` algorithm, you must first execute the following commands to obtain the patched `imu_tk_matlab` source code:
```bash
# Obtain original imu_tk_matlab code
git submodule update
# Apply changes and fixes to imu_tk_matlab
cd calibration/matlab/imu_tk_matlab
git apply ../patches/*.patch
# Start matlab in calibration directory
cd ../../
matlab
```
This should open MATLAB in the `calibration` directory in this repository (the one containing this README file).

In MATLAB you can run the file `matlab/OpenAXES/do_imu_calibration.m` to calculate the calibration coefficients based on one example recording in `log_imu9_2022-06-22_22-13-55.csv`.
Make sure that you are in the correct folder so that the `imu_tk_matlab` source files can be found somewhere in the matlab search path (see `path(...)` calls).

You can change the `filename` variable in `do_imu_calibration.m` in order to calculate calibration coefficients for your own IMUs.


## Applying Calibration Coefficients to IMUs

After calculating the calibration coefficients, the matlab script should print out command line arguments for `imu_control.py`, which can be used to permanently write the calculated coefficients to your IMU.
The following command line should be printed out for the example recording `log_imu9_2022-06-22_22-13-55.csv`:
```
imu_control.py imu9 --write-calibration-full="0.96276 0 0 0.0015751 0.96732 0 -0.00018891 0.000859 0.97079 0.064447 0.011034 0.034253 0.95418 0 0 0.0031573 0.96061 0 0.009235 0.00049713 0.9501 -0.033224 0.080497 0.01579 0.97288 0.0076747 -0.0063582 -0.011829 1.0062 -0.00033403 0.010087 0.00029571 1.0068 -0.00020494 0.00063007 -0.00036962"
```

Since the coefficients are stored in flash memory, they are retained across resets.
When restarting the IMU, the calibration coefficients should automatically be loaded from flash and applied to all measurements (raw data and quaternions), unless disabled via the corresponding GATT interface via `imu_control.py`.

More information about `imu_control.py` (e.g. enabling, disabling, or checking the current calibration status) can be found in [the corresponding README in `/tools/imu_control/`](/tools/imu_control/README.md).


## Contributing Changes to imu_tk_matlab

As mentioned above, the lack of a license in `imu_tk_matlab` project means that we can not include its source code into this repository, just our changes.
If you change anything in `imu_tk_matlab` and want to contribute your changes to the OpenAXES repository, please use the following procedure:

1. Commit your changes by using git inside the submodule folder `imu_tk_matlab`
2. Inside the submodule folder, run `git format-patch origin/master -o ../patches/`
3. Commit all new `.patch` files that were created into the OpenAXES repository.
4. Do _**NOT**_ <s>`git add imu_tk_matlab`</s>




[1]: https://doi.org/10.1109/ICRA.2014.6907297 "Paper 'A robust and easy to implement method for IMU calibration without external equipments' by Tedaldi et al."
[2]: https://github.com/JzHuai0108/imu_tk_matlab "Matlab scripts of David Tedaldi's ICRA14 paper, a robust and easy to implement method for IMU calibration by Jianzhu Huai"
[3]: https://arxiv.org/pdf/2207.04801.pdf "Preprint 'Improved Calibration Procedure for Wireless Inertial
Measurement Units without Precision Equipment' by Webering et al."
[4]: https://bitbucket.org/alberto_pretto/imu_tk "The original imu_tk C++ implementation"