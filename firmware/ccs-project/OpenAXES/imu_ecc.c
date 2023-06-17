/*

@file  imu_ecc.h

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

#include "imu_ecc.h"

static int16_t last_data[IMU_ECC_NUM_SLOTS][IMU_ECC_LENGTH];
static int16_t last_index[IMU_ECC_NUM_SLOTS];
static int16_t last_ecc[IMU_ECC_NUM_SLOTS];

void imu_ecc_reset()
{
    for (int slot = 0; slot < IMU_ECC_NUM_SLOTS; slot++) {
        last_index[slot] = 0;
        last_ecc[slot] = 0;
        for (int item = 0; item < IMU_ECC_LENGTH; item++) {
            last_data[slot][item] = 0;
        }
    }
}

int16_t imu_ecc_next(uint8_t slot, int16_t new_value)
{
    if (slot >= IMU_ECC_NUM_SLOTS) while (1); // hang here to allow debugging
    last_ecc[slot] ^= last_data[slot][last_index[slot]]; // Add the datum from the previous tick to the XOR sum
    last_index[slot] += 1;
    if (last_index[slot] >= IMU_ECC_LENGTH) last_index[slot] = 0; // Advance index to point to the oldest datum
    last_ecc[slot] ^= last_data[slot][last_index[slot]]; // Remove the oldest datum from the XOR sum
    last_data[slot][last_index[slot]] = new_value;   // Overwrite oldest datum with new datum in buffer
    return last_ecc[slot];
}

