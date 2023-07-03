# Hardware

The **Hardware** subdirectory contains the ECAD design files for the OpenAXES PCB and its fabrication outputs, and MCAD files for the housing and calibration fixture.

## PCB

This directory comprises the Altium Designer and KiCad EDA files. 
Altium Designer is a globally renowned PCB and electronic design software. 
KiCad, on the other hand, is a free open source software suite for EDA. 
The use of these two platforms ensures our project is accessible to a broad range of users.

The OpenAXES hardware platform was originally developed using Altium Designer.
To allow for the use of open source EDA software tools it has been ported to KiCad.
Known differences in the design files are only found in the silkscreen layers, so that there are no restrictions on the underlying functionality of the PCB.

### Getting Started

To work with the files contained in this directory, you will need to have the appropriate software installed on your local machine.

- For Altium files, download and install [Altium Designer](https://www.altium.com/)
- For KiCad files, download and install [KiCad EDA](https://www.kicad.org/)

Once you have installed the appropriate software, clone the repository and navigate to the respective subdirectories to access the project files.

### Fabrication outpus

Fabrication outputs are provided in the _PCB/Altium/Project Outputs for OpenAXES_ subdirectory.
Output files for both single PCB and a panel optimized for mass production are included.
 
### Deprecated components

- **BMI160**; pin-compatible replacement: BMI270

## Programming adapter

## Housing

## Calibration fixture
