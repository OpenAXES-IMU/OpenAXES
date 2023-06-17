/******************************************************************************

 @file  simple_gatt_profile.c

 @brief This file contains the Simple GATT profile sample GATT service profile
        for use with the BLE sample application.

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

/*********************************************************************
 * INCLUDES
 */
#include <ti/drivers/UART.h>
#include <string.h>
#include <assert.h>
#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "simple_gatt_profile.h"
#include "OpenAXES/imu_calibration.h"

#ifdef SYSCFG
#include "ti_ble_config.h"

#ifdef USE_GATT_BUILDER
#include "ti_services.h"
#endif

#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
#ifndef USE_GATT_BUILDER

#define SERVAPP_NUM_ATTR_SUPPORTED        21 //prior 17

#ifndef NULL
#define NULL ((void*)0) // Stop CCS C highlighter from complaining
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8 simpleProfileServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 simpleProfilechar1UUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR1_UUID), HI_UINT16(SIMPLEPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 simpleProfilechar2UUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR2_UUID), HI_UINT16(SIMPLEPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 simpleProfilechar3UUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR3_UUID), HI_UINT16(SIMPLEPROFILE_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8 simpleProfilechar4UUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR4_UUID), HI_UINT16(SIMPLEPROFILE_CHAR4_UUID)
};

// Characteristic 5 UUID: 0xFFF5
CONST uint8 simpleProfilechar5UUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SIMPLEPROFILE_CHAR5_UUID), HI_UINT16(SIMPLEPROFILE_CHAR5_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */
//extern UART_Handle uart;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static simpleProfileCBs_t *simpleProfile_AppCBs = NULL;


/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
static CONST gattAttrType_t simpleProfileService = { ATT_BT_UUID_SIZE, simpleProfileServUUID };


// Simple Profile Characteristic 1 Properties
static uint8 simpleProfileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;
// Characteristic 1 Value
static uint8 simpleProfileChar1[SIMPLEPROFILE_CHAR1_LEN] = { 0, 0 };//sensor reading disabled by default
// Simple Profile Characteristic 1 User Description
static uint8 simpleProfileChar1UserDesp[] = "Change IMU Configuration";

//Simple Profile Characteristic 1 Configuration Each client has its own
//instantiation of the Client Characteristic Configuration. Reads of the
//Client Characteristic Configuration only shows the configuration for
//that client and writes only affect the configuration of that client.
//static gattCharCfg_t *simpleProfileChar1Config;


// Simple Profile Characteristic 2 Properties
static uint8 simpleProfileChar2Props = GATT_PROP_NOTIFY | GATT_PROP_READ;
// Characteristic 2 Value
static uint8 simpleProfileChar2[SIMPLEPROFILE_CHAR2_LEN] = {0,0,0,0,0,0,0,0,0,0,0};
// Simple Profile Characteristic 2 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *simpleProfileChar2Config;
// Simple Profile Characteristic 2 User Description
static uint8 simpleProfileChar2UserDesp[] = "IMU status and battery info";


static uint8 simpleProfileChar3Props = GATT_PROP_WRITE;
// Characteristic 3 Value
static uint8 simpleProfileChar3[SIMPLEPROFILE_CHAR3_LEN] = {0,0,};
// Simple Profile Characteristic 3 User Description
static uint8 simpleProfileChar3UserDesp[] = "Reset Sensors";


// Simple Profile Characteristic 4 Properties
static uint8 simpleProfileChar4Props = GATT_PROP_NOTIFY | GATT_PROP_READ;
// Characteristic 4 Value
//static uint8 simpleProfileChar4 = 0;
static uint8 simpleProfileChar4[SIMPLEPROFILE_CHAR4_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// Simple Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *simpleProfileChar4Config;
// Simple Profile Characteristic 4 User Description
static uint8 simpleProfileChar4UserDesp[] = "IMU Values";


// Simple Profile Characteristic 5 Properties
static uint8 simpleProfileChar5Props = GATT_PROP_READ;
// Characteristic 5 Value
static uint8 simpleProfileChar5[SIMPLEPROFILE_CHAR5_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
// Simple Profile Characteristic 5 User Description
static uint8 simpleProfileChar5UserDesp[] = "Firmware version information";


/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simpleProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] =
{
  // Simple Profile Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&simpleProfileService            /* pValue */
  },

    // Characteristic 1 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &simpleProfileChar1Props
    },

      // Characteristic Value 1
      {
        { ATT_BT_UUID_SIZE, simpleProfilechar1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        simpleProfileChar1
      },
      //Characteristic 1 configuration
         /*   {
              { ATT_BT_UUID_SIZE, clientCharCfgUUID },
              GATT_PERMIT_READ | GATT_PERMIT_WRITE,
              0,
              (uint8 *)simpleProfileChar1Config
            },*/
      // Characteristic 1 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        simpleProfileChar1UserDesp
      },

    // Characteristic 2 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &simpleProfileChar2Props
    },

      // Characteristic Value 2
      {
        { ATT_BT_UUID_SIZE, simpleProfilechar2UUID },
        GATT_PERMIT_READ,
        0,
        simpleProfileChar2
      },

      // Characteristic 2 configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&simpleProfileChar2Config
      },

      // Characteristic 2 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        simpleProfileChar2UserDesp
      },

    // Characteristic 3 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &simpleProfileChar3Props
    },

      // Characteristic Value 3
      {
        { ATT_BT_UUID_SIZE, simpleProfilechar3UUID },
        GATT_PERMIT_WRITE,
        0,
        simpleProfileChar3
      },

      // Characteristic 3 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        simpleProfileChar3UserDesp
      },

    // Characteristic 4 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &simpleProfileChar4Props
    },

      // Characteristic Value 4
      {
        { ATT_BT_UUID_SIZE, simpleProfilechar4UUID },
        0,
        0,
        simpleProfileChar4
      },

      // Characteristic 4 configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&simpleProfileChar4Config
      },

      // Characteristic 4 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        simpleProfileChar4UserDesp
      },

    // Characteristic 5 Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &simpleProfileChar5Props
    },

      // Characteristic Value 5
      {
        { ATT_BT_UUID_SIZE, simpleProfilechar5UUID },
        GATT_PERMIT_READ,
        0,
        simpleProfileChar5
      },

      // Characteristic 5 User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        simpleProfileChar5UserDesp
      },
};
#endif // USE_GATT_BUILDER
/*********************************************************************
 * LOCAL FUNCTIONS
 */
bStatus_t simpleProfile_ReadAttrCB(uint16_t connHandle,
                                          gattAttribute_t *pAttr,
                                          uint8_t *pValue, uint16_t *pLen,
                                          uint16_t offset, uint16_t maxLen,
                                          uint8_t method);
bStatus_t simpleProfile_WriteAttrCB(uint16_t connHandle,
                                           gattAttribute_t *pAttr,
                                           uint8_t *pValue, uint16_t len,
                                           uint16_t offset, uint8_t method);
#ifndef USE_GATT_BUILDER
/*********************************************************************
 * PROFILE CALLBACKS
 */

// Simple Profile Service Callbacks
// Note: When an operation on a characteristic requires authorization and
// pfnAuthorizeAttrCB is not defined for that characteristic's service, the
// Stack will report a status of ATT_ERR_UNLIKELY to the client.  When an
// operation on a characteristic requires authorization the Stack will call
// pfnAuthorizeAttrCB to check a client's authorization prior to calling
// pfnReadAttrCB or pfnWriteAttrCB, so no checks for authorization need to be
// made within these functions.
CONST gattServiceCBs_t simpleProfileCBs =
{
  simpleProfile_ReadAttrCB,  // Read callback function pointer
  simpleProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SimpleProfile_AddService( uint32 services )
{
  uint8 status;

  // Allocate Client Characteristic Configuration table
  simpleProfileChar4Config = (gattCharCfg_t *)ICall_malloc( sizeof(gattCharCfg_t) *
                                                            MAX_NUM_BLE_CONNS );
  if ( simpleProfileChar4Config == NULL )
  {
    return ( bleMemAllocError );
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( LINKDB_CONNHANDLE_INVALID, simpleProfileChar4Config );

  if ( services & SIMPLEPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( simpleProfileAttrTbl,
                                          GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &simpleProfileCBs );
  }
  else
  {
    status = SUCCESS;
  }

  // Length of CHAR3 is 200 bytes, so we need to queue 10 write commands at 20 bytes each
  uint8_t new_prepare_write_queue_length = 10;
  status = GATTServApp_SetParameter(GATT_PARAM_NUM_PREPARE_WRITES, 1, &new_prepare_write_queue_length);

  return ( status );
}

/*********************************************************************
 * @fn      SimpleProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t SimpleProfile_RegisterAppCBs( simpleProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    simpleProfile_AppCBs = appCallbacks;

    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      SimpleProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value )
{
    UART_TRACE('P');
  bStatus_t ret = SUCCESS;
  volatile uint8_t urefVal = *(uint8_t*)value;
  switch ( param )
  {
    case SIMPLEPROFILE_CHAR1:
      if ( len == SIMPLEPROFILE_CHAR1_LEN )
      {
        //simpleProfileChar1 = *((uint8*)value);
         // VOID memcpy( simpleProfileChar1, value, SIMPLEPROFILE_CHAR1_LEN );
          VOID osal_memcpy( simpleProfileChar1, value, SIMPLEPROFILE_CHAR1_LEN );
      }
      else
      {
          ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR2:
      if ( len > 0 && len <= SIMPLEPROFILE_CHAR2_LEN )
      {
          VOID osal_memcpy( simpleProfileChar2, value, len );

          // See if Notification has been enabled
          GATTServApp_ProcessCharCfg( simpleProfileChar2Config, simpleProfileChar2, FALSE,
                                      simpleProfileAttrTbl, GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                      INVALID_TASK_ID, simpleProfile_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR4:
      if ( len > 0 && len <= SIMPLEPROFILE_CHAR4_LEN )
      {
          VOID osal_memcpy( simpleProfileChar4, value, len );

        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( simpleProfileChar4Config, simpleProfileChar4, FALSE,
                                    simpleProfileAttrTbl, GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                    INVALID_TASK_ID, simpleProfile_ReadAttrCB );
      }
      else
      {
          ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR5:
      if ( len > 0 && len <= SIMPLEPROFILE_CHAR5_LEN )
      {
        VOID memcpy( simpleProfileChar5, value, len );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      SimpleProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_GetParameter( uint8 param, void *value, size_t count)
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SIMPLEPROFILE_CHAR1:
        assert(count == SIMPLEPROFILE_CHAR1_LEN);
        VOID memcpy( value, simpleProfileChar1, SIMPLEPROFILE_CHAR1_LEN );
      break;

    case SIMPLEPROFILE_CHAR2:
        assert(count == SIMPLEPROFILE_CHAR2_LEN);
        VOID memcpy( value, simpleProfileChar2, SIMPLEPROFILE_CHAR2_LEN );
      break;

    case SIMPLEPROFILE_CHAR3:
        assert(count <= SIMPLEPROFILE_CHAR3_LEN);
        VOID memcpy( value, simpleProfileChar3, count );
      break;

    case SIMPLEPROFILE_CHAR4:
        assert(count == SIMPLEPROFILE_CHAR4_LEN);
        VOID memcpy( value, simpleProfileChar4, SIMPLEPROFILE_CHAR4_LEN );
      break;

    case SIMPLEPROFILE_CHAR5:
        assert(count == SIMPLEPROFILE_CHAR5_LEN);
        VOID memcpy( value, simpleProfileChar5, SIMPLEPROFILE_CHAR5_LEN );
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}
#endif // USE_GATT_BUILDER
/*********************************************************************
 * @fn          simpleProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
bStatus_t simpleProfile_ReadAttrCB(uint16_t connHandle,
                                    gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t *pLen,
                                    uint16_t offset, uint16_t maxLen,
                                    uint8_t method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1, 2 and 5 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
    // characteristic 4 does not have read permissions, but because it
    //   can be sent as a notification, it is included here
      case SIMPLEPROFILE_CHAR1_UUID:
          *pLen = SIMPLEPROFILE_CHAR1_LEN;
          VOID osal_memcpy( pValue, pAttr->pValue, SIMPLEPROFILE_CHAR1_LEN );
        break;
      case SIMPLEPROFILE_CHAR2_UUID:
          *pLen = SIMPLEPROFILE_CHAR2_LEN;
          VOID osal_memcpy( pValue, pAttr->pValue, SIMPLEPROFILE_CHAR2_LEN );
       break;
      case SIMPLEPROFILE_CHAR4_UUID:
          *pLen = SIMPLEPROFILE_CHAR4_LEN;
          VOID memcpy( pValue, pAttr->pValue, SIMPLEPROFILE_CHAR4_LEN );
        break;

      case SIMPLEPROFILE_CHAR5_UUID:
        *pLen = SIMPLEPROFILE_CHAR5_LEN;
        VOID memcpy( pValue, pAttr->pValue, SIMPLEPROFILE_CHAR5_LEN );
        break;

      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      simpleProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
bStatus_t simpleProfile_WriteAttrCB(uint16_t connHandle,
                                     gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint16_t len,
                                     uint16_t offset, uint8_t method)
  {
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      /*case SIMPLEPROFILE_CHAR1_UUID:
          uint8 *pCurValue1 = (uint8 *)pAttr->pValue;
        *pCurValue1 = pValue[0];
          notifyApp = SIMPLEPROFILE_CHAR1;
          break;*/
    case SIMPLEPROFILE_CHAR1_UUID:

   //Validate the value
   //Make sure it's not a blob oper
    if ( offset == 0 )
    {
      if ( len != SIMPLEPROFILE_CHAR1_LEN )
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_LONG;
    }

   //Write the value
    if ( status == SUCCESS )
    {
      VOID osal_memcpy( pAttr->pValue, pValue, SIMPLEPROFILE_CHAR1_LEN );
      notifyApp = SIMPLEPROFILE_CHAR1;
    }

    break;
      case SIMPLEPROFILE_CHAR3_UUID:
        if (len == 0)  // don't accept empty writes (not sure if this can happen)
        {
            status = ATT_ERR_INVALID_VALUE_SIZE;
            break; // don't access pValue[0] if len == 0
        }

        uint8_t command = pValue[0]; // First byte always encodes the action that should be performed

        uint8_t payload_length;
        if (len <= 1) // len==0 should never occur
        {
            payload_length = 0; // Single-byte writes have no payload or length field, only a command
        }
        else if (len > 1) // Multi-byte writes have a 2-byte header [command,length] followed by payload.
        {
            payload_length = len - CHAR3_HEADER_LENGTH; // Note that this could also be a single-byte command with len==CHAR3_HEADER_LENGTH.
        }

        // Check all the things that could go wrong and reject the write
        if (offset > 0) // Only accept prepared writes that write all data at once
        {
            status = ATT_ERR_INVALID_OFFSET;
        }
        else if (len > SIMPLEPROFILE_CHAR3_LEN)  // don't write past the end of the characteristic
        {
            status = ATT_ERR_INVALID_VALUE_SIZE;
        }
        else if (len > 1 && pValue[1] != len) // For multi-byte writes, the second byte is a length field
        {
            status = ATT_ERR_INVALID_VALUE_SIZE; // The length field must match the amount of data we get
        }
        // For known commands, check that the number of bytes is as expected
        else if (command <= IMU_RESET_MAX_BIT_VALUE && payload_length != 0)
        {
            status = ATT_ERR_INVALID_VALUE; // Reset commands never have payload
        }
        else if (command == IMU_CONFIG_CALIBRATION_COEFFICIENTS
                && payload_length != IMU_CONFIG_CALIBRATION_COEFFICIENTS_PAYLOAD_LENGTH)
        {
            status = ATT_ERR_INVALID_VALUE;
        }
        else if (command == IMU_CONFIG_SAVE_CALIBRATION
                && payload_length != IMU_CONFIG_SAVE_CALIBRATION_PAYLOAD_LENGTH)
        {
            status = ATT_ERR_INVALID_VALUE;
        }
        else if (command == IMU_CONFIG_LOAD_CALIBRATION
                && payload_length != IMU_CONFIG_LOAD_CALIBRATION_PAYLOAD_LENGTH)
        {
            status = ATT_ERR_INVALID_VALUE;
        }
        else if (command == IMU_CONFIG_USE_CALIBRATION
                && payload_length != IMU_CONFIG_USE_CALIBRATION_PAYLOAD_LENGTH)
        {
            status = ATT_ERR_INVALID_VALUE;
        }
        else if (command == IMU_CONFIG_ENABLE_UART_TRANSMISSION
                && payload_length != IMU_CONFIG_ENABLE_UART_TRANSMISSION_PAYLOAD_LENGTH)
        {
            status = ATT_ERR_INVALID_VALUE;
        }

        if ( status != SUCCESS ) break;

        //Write the value
        VOID osal_memcpy( pAttr->pValue, pValue, len );
        if (len == 1) {
            pAttr->pValue[1] = CHAR3_HEADER_LENGTH; // Explicitly set length field of header to zero payload if only 1 byte was written
        }
        notifyApp = SIMPLEPROFILE_CHAR3;

        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;

      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a characteristic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && simpleProfile_AppCBs && simpleProfile_AppCBs->pfnSimpleProfileChange )
  {
    simpleProfile_AppCBs->pfnSimpleProfileChange( notifyApp );
  }

  return ( status );
}

/*********************************************************************
*********************************************************************/
