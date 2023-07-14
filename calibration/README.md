# OpenAXES Calibration Tools and Example Dataset

The `calibration` subdirectory contains the scripts used to calculate the calibration coefficients for OpenAXES IMUs (or any other suitable IMU) using the method published by Tedaldi et al. in their 2014 paper [A robust and easy to implement method for IMU calibration without external equipments][1]

For calibrating our own IMUs we [used the MATLAB implementation by Jianzhu Huai][2] (state as of 2017).
The `imu_tk_matlab` code is included in this repository as a submodule only, because it has no license that would permit us to include a copy of the code itself.
We made some changes, improvements, and bugfixes to the original code, which are contained in the [patches](/calibration/matlab/patches/) subdirectory.
The changes are described in the `CHANGES.md` inside the patch, as well as in greater detail in [our preprint on arxiv.org][3].

Alternatively, one could try to use [the original C++ implementation by Tedaldi et al.][4], which is released under the BSD license.
We used the MATLAB version because it was easier to get started.


## Calibration Data Recording

We recommend using the icosahedral calibration fixture for reliable and repeatable recording of calibration data for the OpenAXES IMU.
The 3D-printable model can be found in [the /hardware/calibrator/ directory](/hardware/calibrator/D20-Calibrator.3mf) in 3MF format for PrusaSlicer.

Detailed step-by-step instructions for recording a calibration sequence can be found in [the dataset README file](/calibration/openaxes_example_calibration_dataset/README.md#dataset-recording).


## Calculating Calibration Coefficients

To use the MATLAB implementation of the `imu_tk` algorithm, you must first execute the following commands to obtain the patched `imu_tk_matlab` source code:
```bash
git submodule update
cd calibration/matlab/imu_tk_matlab
git apply ../patches/*.patch
```
Then you can open MATLAB and navigate to the `calibration` directory in this repository (the one containing this README.md).

In MATLAB you can run the file `matlab/OpenAXES/do_imu_calibration.m` to calculate the calibration coefficients based on one example recording in `log_imu9_2022-06-22_22-13-55.csv`.
Read and adapt the file as necessary in order to calculate calibration coefficients for your own IMUs.


## Patching imu_tk_matlab

As mentioned above, the lack of a license in `imu_tk_matlab` project means that we can not include its source code into this repository, just our changes.
If you change anything in `imu_tk_matlab` and want to contribute your changes to the OpenAXES repository, please use the following procedure:

1. Commit your changes by using git inside the submodule folder `imu_tk_matlab`
2. Inside the submodule folder, run `git format-patch origin/master -o ../patches/`
3. Commit all new `.patch` files that were created into the OpenAXES repository.
4. Do _**NOT**_ <s>`git add imu_tk_matlab`</s>


## Example Calibration Dataset

The subdirectory `/calibration/openaxes_example_calibration_dataset` contains example calibration recordings from four OpenAXES IMUs.
Please refer to the [separate README file for the dataset](/calibration/openaxes_example_calibration_dataset/README.md) for more information.

**Since git repositories are not suitable for storing large amounts of static data, the dataset itself is kept in the research data repository of the LUH: https://doi.org/10.25835/oimsodoy. Download the dataset from there and extract it into the `/calibration` directory in this repository.**



[1]: https://doi.org/10.1109/ICRA.2014.6907297 "Paper 'A robust and easy to implement method for IMU calibration without external equipments' by Tedaldi et al."
[2]: https://github.com/JzHuai0108/imu_tk_matlab "Matlab scripts of David Tedaldi's ICRA14 paper, a robust and easy to implement method for IMU calibration by Jianzhu Huai"
[3]: https://arxiv.org/pdf/2207.04801.pdf "Preprint 'Improved Calibration Procedure for Wireless Inertial
Measurement Units without Precision Equipment' by Webering et al."
[4]: https://bitbucket.org/alberto_pretto/imu_tk "The original imu_tk C++ implementation"