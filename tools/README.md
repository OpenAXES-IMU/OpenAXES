# OpenAXES IMU Tools

This directory contains scripts to communicate with, control, and configure the OpenAXES IMU.

This documentation describes Open Hardware and is licensed under the CERN-OHL-S v2 or any later version. See [License](#license) below for more information.

# tools/imu_control

The `imu_control` script can be used to control the OpenAXES IMU via the Bluetooth Low Energy (BLE) GATT interface.
It also serves as a demonstration how to implement other clients for the GATT interface.

The main features of the script are:
* selecting which measurement data the IMU should send
* receiving measurement data from the IMU via BLE notifications
* recording the measurement data in CSV files
* passing the IMU data on to other processes via UDP
* showing status information like battery state or firmware version
* configuring the IMU, for example the calibration coefficients
* assistive functions for recording calibration datasets


The main documentation for `imu_control.py` can be found in **[`/tools/imu_control/README.md`](/tools/imu_control/README.md)**.


# License

Copyright 2023 Nils Stanislawski and Fritz Webering

This documentation describes Open Hardware and is licensed under the CERN-OHL-S-v2 or any later version.

You may redistribute and modify this documentation under the terms of the CERN OHL-S-v2 (https://ohwr.org/cern_ohl_s_v2.txt) or any later version. This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN OHL v2-S for applicable conditions

Source Location: https://github.com/IMS-AS-LUH/OpenAXES

For detailed license information on individual files, please see the file [LICENSE.txt](/LICENSE.txt) in the root folder of this repository.
