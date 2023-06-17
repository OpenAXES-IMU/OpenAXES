/******************************************************************************

 @file  simple_gatt_profile.h

 @brief This file contains the Simple GATT profile definitions and prototypes
        prototypes.

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2010-2020, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

#ifndef SIMPLEGATTPROFILE_H
#define SIMPLEGATTPROFILE_H

#ifdef __cplusplus
extern "C"
{
#endif

// Compile-time check if the size of a struct is greater than a maximum
#define SIZE_CHECK( sname, maxsize ) typedef  char sname ## _size_check_dummy [ 1 - 2* !!(sizeof(sname) > (maxsize)) ]

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * TYPEDEFS
 */

typedef struct __attribute__((__packed__)) {
    int16_t battery_current;            ///< Battery current in steps of 156.25 uA per LSB (with 0.01 Ohm sense resistor)
    int16_t battery_current_average;    ///< Battery current (averaged over time) in steps of 156.25 uA per LSB (with 0.01 Ohm sense resistor)
    uint16_t battery_voltage;           ///< Battery voltage in steps of 0.078125 mV per LSB
    uint16_t battery_capacity;          ///< Battery remaining capacity in units of 0.5 mAh per LSB (with 0.01 Ohm sense resistor)
    uint16_t battery_capacity_full;     ///< Battery full capacity in units of 0.5 mAh per LSB (with 0.01 Ohm sense resistor)
    uint8_t battery_percent;            ///< Battery state of charge in percent
    uint8_t calibration_status;         ///< Bitfield: Status of IMU calibration. See CHAR2_CALIBRATION_STATUS_* macros for meaning
} char2_contents_t;


/*********************************************************************
 * CONSTANTS
 */
#ifndef USE_GATT_BUILDER
// Profile Parameters
#define SIMPLEPROFILE_CHAR1                   0  // RW uint8 - Profile Characteristic 1 value
#define SIMPLEPROFILE_CHAR2                   1  // RW uint8 - Profile Characteristic 2 value
#define SIMPLEPROFILE_CHAR3                   2  // RW uint8 - Profile Characteristic 3 value
#define SIMPLEPROFILE_CHAR4                   3  // RW uint8 - Profile Characteristic 4 value
#define SIMPLEPROFILE_CHAR5                   4  // RW uint8 - Profile Characteristic 5 value

// Simple Profile Service UUID
#define SIMPLEPROFILE_SERV_UUID               0xAD42

// Key Pressed UUID
#define SIMPLEPROFILE_CHAR1_UUID            0xCF01 //Turn on/off Sensor Reading
#define SIMPLEPROFILE_CHAR2_UUID            0xCF02 //IMU and battery information
#define SIMPLEPROFILE_CHAR3_UUID            0xCF03 //General configuration (write-only)
#define SIMPLEPROFILE_CHAR4_UUID            0xCF04 //3d position (usually for GPS, this does not comply with standards!
#define SIMPLEPROFILE_CHAR5_UUID            0xCF05 //version info

// Simple Keys Profile Services bit fields
#define SIMPLEPROFILE_SERVICE               0x00000001

// Length of Characteristics in bytes
#define SIMPLEPROFILE_CHAR1_LEN           2
#define SIMPLEPROFILE_CHAR2_LEN           (sizeof(char2_contents_t))
SIZE_CHECK(char2_contents_t, 20); // Fail to compile if sizeof char2_contents_t > 20
#define SIMPLEPROFILE_CHAR3_LEN           200
#define SIMPLEPROFILE_CHAR4_LEN           20
#define SIMPLEPROFILE_CHAR5_LEN           20


// IMU Reset bits masks for CHAR3. Must be <= 0x0F
#define IMU_RESET_MAX_BIT_VALUE  0x07
#define IMU_RESET_SENSORS_BM     0x01
#define IMU_RESET_QUATERNION_BM  0x02
#define IMU_RESET_INDEX_BM       0x04

// IMU config commands for CHAR3. Must be >= 0x10
#define CHAR3_HEADER_LENGTH   2
#define IMU_CONFIG_COMMAND_MIN_IDENTIFIER             (IMU_RESET_VALUE_MAX + 1)

#define IMU_CONFIG_CALIBRATION_COEFFICIENTS                 0x10
#define IMU_CONFIG_CALIBRATION_COEFFICIENTS_PAYLOAD_LENGTH  (sizeof(calibration_data_t))

#define IMU_CONFIG_SAVE_CALIBRATION                 0x11
#define IMU_CONFIG_SAVE_CALIBRATION_PAYLOAD_LENGTH  0

#define IMU_CONFIG_LOAD_CALIBRATION                 0x12
#define IMU_CONFIG_LOAD_CALIBRATION_PAYLOAD_LENGTH  0

#define IMU_CONFIG_USE_CALIBRATION                 0x13
#define IMU_CONFIG_USE_CALIBRATION_PAYLOAD_LENGTH  1  // boolean true == enable, false == disable

#define IMU_CONFIG_ENABLE_UART_TRANSMISSION                 0x14
#define IMU_CONFIG_ENABLE_UART_TRANSMISSION_PAYLOAD_LENGTH  1  // boolean true == enable, false == disable

#define CHAR2_CALIBRATION_STATUS_ENABLED        0x01
#define CHAR2_CALIBRATION_STATUS_ERROR_SAVE     0x20
#define CHAR2_CALIBRATION_STATUS_ERROR_LOAD     0x40
#define CHAR2_CALIBRATION_STATUS_UNINITIALIZED  0x80

/*********************************************************************
 * MACROS
 */

// Endian conversion. Bluetooth transmissions are big endian.
#ifdef __LITTLE_ENDIAN__
#define TO_BE16(_a_)        __builtin_bswap16 (_a_)
#define TO_BE32(_a_)        __builtin_bswap32 (_a_)
#define FROM_BE16(_a_)      __builtin_bswap16 (_a_)
#define FROM_BE32(_a_)      __builtin_bswap32 (_a_)
#else
#define TO_BE16(_a_)        (_a_)
#define TO_BE32(_a_)        (_a_)
#define FROM_BE16(_a_)      (_a_)
#define FROM_BE32(_a_)      (_a_)
#endif

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*simpleProfileChange_t)( uint8 paramID );

typedef struct
{
  simpleProfileChange_t        pfnSimpleProfileChange;  // Called when characteristic value changes
} simpleProfileCBs_t;



/*********************************************************************
 * API FUNCTIONS
 */


/*
 * SimpleProfile_AddService- Initializes the Simple GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t SimpleProfile_AddService( uint32 services );

/*
 * SimpleProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t SimpleProfile_RegisterAppCBs( simpleProfileCBs_t *appCallbacks );

/*
 * SimpleProfile_SetParameter - Set a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value );

/*
 * SimpleProfile_GetParameter - Get a Simple GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t SimpleProfile_GetParameter( uint8 param, void *value, size_t count);


/*********************************************************************
*********************************************************************/
#endif // USE_GATT_BUILDER

#ifdef __cplusplus
}
#endif

#endif /* SIMPLEGATTPROFILE_H */
