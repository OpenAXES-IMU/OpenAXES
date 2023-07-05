![OpenAXES Graphical Abstract](docs/static/images/gagraphic.png)

# OpenAXES

**Hardware design files and source code for an open-access wireless IMU** developed at the [Institute of Microelectronic Systems](https://www.ims.uni-hannover.de/) at the Leibniz University Hannover, Germany.

The OpenAXES IMU is open-source hardware licensed under the CERN-OHL-S-v2+. This means that you are allowed to modify, share, or sell the design, **if** you adhere to the respecitive licenses (see section [License](#license) or [LICENSE.txt](/LICENSE.txt) for details).

If you use the OpenAXES project (or parts of it) in scientific work, please see the [Citation](#citation) section below.

## Hardware

The hardware subdirectory contains the ECAD design files for the OpenAXES PCB and its fabrication outputs, and MCAD files for the housing, the programming adapter, and a calibration fixture.

More information about the hardware and how to fabricate it can be found in [the README in the hardware folder](/hardware/README.md)

## Firmware

The OpenAXES firmware is written in C and based on the CC2652 SDK from Texas Instruments.
It is based on TI-RTOS and can be compiled with [Code Composer Studio (CCS)](https://www.ti.com/tool/CCSTUDIO).

More information on how to compile and flash the firmware can be found in [the README in the firmware folder](/firmware/README.md)

## Citation

If you use the OpenAXES project (or parts of it) in scientific work,
please cite our upcoming paper (**submitted for peer review - not published yet**).

> Title: "OpenAXES: An Open-Source Wireless Inertial Measurement Unit"
>
> Authors: Fritz Webering, Nils Stanislawski, Joseph Winkler, Sarah Kleinjohann, Holger Blume
>
> Journal: [IEEE Sensors Letters](https://ieee-sensors.org/ieee-sensors-letters/)

The correct precise citation will be updated as soon as possible.


# License

Copyright 2023 Nils Stanislawski (nils.stanislawski@ims.uni-hannover.de) and Fritz Webering (fritz.webering@ims.uni-hannover.de)

This documentation describes Open Hardware and is licensed under the CERN-OHL-S-v2 or any later version.

You may redistribute and modify this documentation under the terms of the CERN OHL-S-v2 (https://ohwr.org/cern_ohl_s_v2.txt) or any later version. This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE. Please see the CERN OHL v2-S for applicable conditions

Source Location: https://github.com/IMS-AS-LUH/OpenAXES

For detailed license information on individual files, please see the file [LICENSE.txt](/LICENSE.txt) in the root folder of this repository.