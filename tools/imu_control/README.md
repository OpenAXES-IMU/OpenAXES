# OpenAXES IMU Tools

This directory contains scripts to communicate with, control, and configure the OpenAXES IMU.

This documentation describes Open Hardware and is licensed under the CERN-OHL-S v2 or any later version. See [License](#license) below for more information.

# Usage of imu_control.py

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


## Prerequisites

* Use a Linux PC (preferably a modern Ubuntu) or VirtualBox and a Bluetooth adapter.
  The ASUS USB-BT400 stick works best in our experience, especially for many simultaneous connections to different IMUs.
  If you use VirtualBox, you can pass through the USB stick to the guest operating system.
* Make sure that gatttool is installed (probably `sudo apt install bluez bluez-tools`)
* Set permissions on hcitool:
  ```
  sudo setcap 'cap_net_raw,cap_net_admin+eip' /usr/bin/hcitool
  ```
  This allows normal users to scan for BLE devices using `hcitool lescan`.
  The udpstream script uses this to set the connection interval using `hcitool lecup ...` to avoid package loss.

## Configuration files

All operations that interact with an IMU expect a BLE MAC address on the command line.
If you do not like to memorize MAC addresses, you can assign arbitrary names to your IMUs and store them in a file called `my_imu_addresses.json` in the following format:
```json
{
    "imu1": "18:04:ED:C4:75:01",
    "imu2": "18:04:ED:C4:75:02"
}
```
These IMU names can then be used on the command line as shown in the examples below.

## Initialize IMUs before each use

* Switch IMUs on.
* Lay IMUs *perfectly still* on a table (LED side up).
* Run `./imu_control.py --reset-sensors imuX` for each IMU. *Do not touch IMUs before all commands completed!*
* IMU initialization is complete. You can now capture data.
  For example, run `./imu_control.py imuX` for each IMU to capture quaternions.
  Make a small pause between starting different IMUs, for example like this:

  ```
  ../imu_control.py imu1
  sleep 0.5
  ../imu_control.py imu2
  sleep 0.5
  ../imu_control.py imu6
  ```

## Lost packets at the start

* At the start of the transmission, some packets may be lost.
  This is normal, because the connection interval is set only a short while after the transmission starts.
* You can skip the first few packets and start recording once the connection is stable:
  `./imu_control.py imuX --skip 100` will skip the first 100 received packets before recording, which is about 1 second.
  Skipped packets will not be reported as "lost".

## Change what type of data is sent

* Reset IMUs (see above).
* Capture data using the `--mode` option.
  For example, `./imu_control.py --mode INDEX,ACCEL_BMI160,GYRO` will record a 16-bit packet index, plus gyroscope and accelerometer data from the BMI160 chip.
* See `./imu_control.py --help` for available data types for `--mode`.

## Battery level

* Battery level is permanently shown by the red blinking LED.
  The duty cycle is equal to the battery level (e.g. 40% on and 60% off means 40% battery level remaining).
* `./imu_control.py imuX -b` or `--battery` will print the battery level before each measurement.
* A WARNING will be printed if the battery level is below 15% when starting a measurement.
* `./imu_control.py imuX --status` will print detailed battery information, like current consumption, voltage, etc.
* The battery is charged via Micro USB. The blue charging LED will turn off when the battery is full.

## Enable/disable calibration

* Calibration coefficients are applied to the raw data by default, if calibration data is found in IMU flash memory.
* Run `./imu_control.py imuX --status` to see `calibration_status`.
  Value `0x00` means disabled, `0x01` means enabled, and anything else is an error.
* Disable use of calibration coefficients (until IMU reboot):
  `./imu_control.py imuX --command USE_CALIBRATION 0` to disable calibration.
* Run `./imu_control.py --command USE_CALIBRATION 1` to enable calibration.
* Permanently reset all calibration coefficients for this IMU:
  `./imu_control.py imuX --write-calibration-full '1 0 0  0 1 0  0 0 1   0 0 0   1 0 0  0 1 0  0 0 1   0 0 0   1 0 0  0 1 0  0 0 1   0 0 0'`.
  The numbers are three 3x3 identity calibration matrices and three 3-element zero bias vectors.

# Usage of bleak_imu_control.py

This script is an experimental port of an old version of `imu_control.py` to use Python 3 and [the python package `bleak`](https://bleak.readthedocs.io/en/latest/) instead of `gatttool` invocations.
Since we could not get the `bleak` interface to work reliably with our IMU, we suspended the experiment and continued using the `gatttool` version that worked more or less reliably.

The current version of the script is presented here only for documentation purposes.
It could be used as a starting point in case someone wants to resume the effort of porting to Python 3 with `bleak`.

# License

Copyright 2023 Nils Stanislawski and Fritz Webering

This documentation describes Open Hardware and is licensed under the CERN-OHL-S-v2 or any later version.

You may redistribute and modify this documentation under the terms of the CERN OHL-S-v2 (https://ohwr.org/cern_ohl_s_v2.txt) or any later version. This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN OHL v2-S for applicable conditions

Source Location: https://github.com/IMS-AS-LUH/OpenAXES

For detailed license information on individual files, please see the file [LICENSE.txt](/LICENSE.txt) in the root folder of this repository.
