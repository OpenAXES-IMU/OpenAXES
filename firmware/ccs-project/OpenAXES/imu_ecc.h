/*

@file  imu_ecc.c

@license  LGPL-3+

This file is part of the OpenAXES project, a wireless IMU.
Copyright 2023 Nils Stanislawski and Fritz Webering

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

*/


#ifndef APPLICATION_IMU_ECC_H_
#define APPLICATION_IMU_ECC_H_

#define IMU_ECC_NUM_SLOTS   3
#define IMU_ECC_LENGTH      15

#include <stdint.h>

void imu_ecc_reset();
int16_t imu_ecc_next(uint8_t ecc_slot, int16_t value);


#endif /* APPLICATION_IMU_ECC_H_ */
