/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: cc13x2_26x2

 ******************************************************************************
 
 Copyright (c) 2013-2020, Texas Instruments Incorporated
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
#include <ti/drivers/Timer.h>
#include <ti/drivers/GPIO.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>

#if !(defined __TI_COMPILER_VERSION__)
#include <intrinsics.h>
#endif

#include <ti/drivers/utils/List.h>

#include <icall.h>
#include "util.h"
#include <bcomdef.h>
/* This Header file contains all BLE API and icall structure definition */
#include <icall_ble_api.h>

#include "Profiles/devinfoservice.h"
#include "Profiles/batteryservice.h"
#include "Profiles/simple_gatt_profile.h"

#ifdef USE_RCOSC
#include <rcosc_calibration.h>
#endif //USE_RCOSC

#include <ti_drivers_config.h>

#include "simple_peripheral.h"
#include "ti_ble_config.h"

#ifdef PTM_MODE
#include "npi_task.h"               // To allow RX event registration
#include "npi_ble.h"                // To enable transmission of messages to UART
#include "icall_hci_tl.h"   // To allow ICall HCI Transport Layer
#endif // PTM_MODE

#include <math.h> //for fminf for quaterntion BLE preparation

#include "OpenAXES/vectormath.h"
#include "Madgwick/madgwick_original.h"
#include "OpenAXES/imu_ecc.h"
#include "OpenAXES/imu_calibration.h"

#include "scif.h"

#include "build_version.h"
#include "OpenAXES/OpenAXES_hardware.h"

/*********************************************************************
 * MACROS
 */

#define GPIO_LED_on(index) GPIO_write(index, 0)
#define GPIO_LED_off(index) GPIO_write(index, 1)


// Debugging macro to use printf on UART
#define UART_printf(fmt, ...)   do { char buffer[255]; sprintf(buffer, fmt, __VA_ARGS__); UART_write(uart, buffer, strlen(buffer)); } while(0)
// Debugging macro to replace Display_printf with if needed
#define UART_Display_printf(display_handle, line, column, fmt, ...)   UART_printf(fmt, __VA_ARGS__)
// Redirect all Display_printf to UART... may or may not be very helpful
// #define Display_printf  UART_Display_printf

// Display Interface
#undef Display_printf
#undef Display_clearLine
#undef Display_clearLines
#define Display_printf(display_handle, line, column, fmt, ...) // Disable all Display_printf
#define Display_clearLine(...) // Disable all Display_clearLine
#define Display_clearLines(...) // Disable all Display_clearLines

/*********************************************************************
 * CONSTANTS
 */
//IMU
//Transmission Enable Bitmasks
#define IMU_INDEX_EN_BM                      0x8000
#define IMU_QUATERNION_EN_BM                 0x4000
#define IMU_ACCEL_ADXL355_EN_BM              0x2000
#define IMU_ACCEL_BMI160_EN_BM               0x1000
#define IMU_GYROS_EN_BM                      0x0800
#define IMU_MAGNET_EN_BM                     0x0400
#define IMU_EXT_ANALOG_EN_BM                 0x0200
#define IMU_TEMP_EN_BM                       0x0100
#define IMU_PRES_EN_BM                       0x0080
#define IMU_GYRO_ECC_EN_BM                   0x0040

#define IMU_SELECT_FILTER_ACCEL_BM           0x0002
#define IMU_ENABLE_SC_BM                     0x0001

//transmission frequency in ms, max 1ms (fixed magdwick filter update rate)
#define IMU_TRANSM_FREQ                      1
#define IMU_SC_SAMPLE_RATE                   100

#define QUATERNION_SCALING_FACTOR           (32767.0f)
#define GYRO_SCALING_FACTOR                 ((float)(180.0 * 16.384 / 3.14159265358979))
#define ACCEL_BMI160_SCALING_FACTOR         (1600.f)
#define ACCEL_ADXL355_SCALING_FACTOR        (1600.f)

// Non-volatile storage (flash) IDs
#define SNV_ID_CALIBRATION_DATA     BLE_NVID_CUST_START // The SNV (Flash memory) id where calibration data is stored.

// How often to perform periodic event (in ms)
#define SP_PERIODIC_EVT_PERIOD               5000

// Task configuration
#define SP_TASK_PRIORITY                     1

#ifndef SP_TASK_STACK_SIZE
#define SP_TASK_STACK_SIZE                   1024
#endif

// Application events
#define SP_STATE_CHANGE_EVT                  0
#define SP_CHAR_CHANGE_EVT                   1
#define SP_ADV_EVT                           3
#define SP_PAIR_STATE_EVT                    4
#define SP_PASSCODE_EVT                      5
#define SP_PERIODIC_EVT                      6
#define SP_READ_RPA_EVT                      7
#define SP_SEND_PARAM_UPDATE_EVT             8
#define SP_SENSOR_CONTROLLER_UPDATE          10

// Internal Events for RTOS application
#define SP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30

// Bitwise OR of all RTOS events to pend on
#define SP_ALL_EVENTS                        (SP_ICALL_EVT             | \
                                              SP_QUEUE_EVT)

// Size of string-converted device address ("0xXXXXXXXXXXXX")
#define SP_ADDR_STR_SIZE     15

// Row numbers for two-button menu
#define SP_ROW_SEPARATOR_1   (TBM_ROW_APP + 0)
#define SP_ROW_STATUS_1      (TBM_ROW_APP + 1)
#define SP_ROW_STATUS_2      (TBM_ROW_APP + 2)
#define SP_ROW_CONNECTION    (TBM_ROW_APP + 3)
#define SP_ROW_ADVSTATE      (TBM_ROW_APP + 4)
#define SP_ROW_RSSI          (TBM_ROW_APP + 5)
#define SP_ROW_IDA           (TBM_ROW_APP + 6)
#define SP_ROW_RPA           (TBM_ROW_APP + 7)
#define SP_ROW_DEBUG         (TBM_ROW_APP + 8)
#define SP_ROW_AC            (TBM_ROW_APP + 9)

// For storing the active connections
#define SP_RSSI_TRACK_CHNLS        1            // Max possible channels can be GAP_BONDINGS_MAX
#define SP_MAX_RSSI_STORE_DEPTH    5
#define SP_INVALID_HANDLE          0xFFFF
#define RSSI_2M_THRSHLD           -30
#define RSSI_1M_THRSHLD           -40
#define RSSI_S2_THRSHLD           -50
#define RSSI_S8_THRSHLD           -60
#define SP_PHY_NONE                LL_PHY_NONE  // No PHY set
#define AUTO_PHY_UPDATE            0xFF

// Spin if the expression is not true
#define SIMPLEPERIPHERAL_ASSERT(expr) if (!(expr)) simple_peripheral_spin();

/*********************************************************************
 * TYPEDEFS
 */

// Auto connect availble groups
enum
{
  AUTOCONNECT_DISABLE = 0,              // Disable
  AUTOCONNECT_GROUP_A = 1,              // Group A
  AUTOCONNECT_GROUP_B = 2               // Group B
};


// App event passed from stack modules. This type is defined by the application
// since it can queue events to itself however it wants.
typedef struct
{
  uint8_t event;                // event type
  void    *pData;               // pointer to message
} spEvt_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPairStateCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t state;
  uint16_t connHandle;
  uint8_t status;
} spPairStateData_t;

// Container to store passcode data when passing from gapbondmgr callback
// to app event. See the pfnPasscodeCB_t documentation from the gapbondmgr.h
// header file for more information on each parameter.
typedef struct
{
  uint8_t deviceAddr[B_ADDR_LEN];
  uint16_t connHandle;
  uint8_t uiInputs;
  uint8_t uiOutputs;
  uint32_t numComparison;
} spPasscodeData_t;

// Container to store advertising event data when passing from advertising
// callback to app event. See the respective event in GapAdvScan_Event_IDs
// in gap_advertiser.h for the type that pBuf should be cast to.
typedef struct
{
  uint32_t event;
  void *pBuf;
} spGapAdvEventData_t;

// Container to store information from clock expiration using a flexible array
// since data is not always needed
typedef struct
{
  uint8_t event;                //
  uint8_t data[];
} spClockEventData_t;

// List element for parameter update and PHY command status lists
typedef struct
{
  List_Elem elem;
  uint16_t  connHandle;
} spConnHandleEntry_t;

// Connected device information
typedef struct
{
  uint16_t         	    connHandle;                        // Connection Handle
  spClockEventData_t*   pParamUpdateEventData;
  Clock_Struct*    	    pUpdateClock;                      // pointer to clock struct
  int8_t           	    rssiArr[SP_MAX_RSSI_STORE_DEPTH];
  uint8_t          	    rssiCntr;
  int8_t           	    rssiAvg;
  bool             	    phyCngRq;                          // Set to true if PHY change request is in progress
  uint8_t          	    currPhy;
  uint8_t          	    rqPhy;
  uint8_t          	    phyRqFailCnt;                      // PHY change request count
  bool             	    isAutoPHYEnable;                   // Flag to indicate auto phy change
} spConnRec_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

static uint8_t battery_level = 255;

// Timers
Timer_Handle    timer_handle_led;
Timer_Params    timer_params_led;

// Task configuration
Task_Struct spTask;
#if defined __TI_COMPILER_VERSION__
#pragma DATA_ALIGN(spTaskStack, 8)
#else
#pragma data_alignment=8
#endif
uint8_t spTaskStack[SP_TASK_STACK_SIZE];

#define APP_EVT_EVENT_MAX 0x9
char *appEventStrings[] = {
  "APP_STATE_CHANGE_EVT     ",
  "APP_CHAR_CHANGE_EVT      ",
  "APP_KEY_CHANGE_EVT       ",
  "APP_ADV_EVT              ",
  "APP_PAIR_STATE_EVT       ",
  "APP_PASSCODE_EVT         ",
  "APP_READ_RPA_EVT         ",
  "APP_PERIODIC_EVT         ",
  "APP_SEND_PARAM_UPDATE_EVT",
  "APP_CONN_EVT             ",
};

// estimated orientation quaternion elements with initial conditions
Quaternion current_q = {1, 0, 0, 0};

Vector3 accel_adxl355_calibrated;
Vector3 accel_bmi160_calibrated;
Vector3 accel_for_filter;
Vector3 accel_accumulator;
Vector3 gyro_raw;
Vector3 gyro_radian;


uint8_t reset_accel_average_counter = 0;
uint16_t sensor_set_index = 0;
uint16_t transmission_counter = 0;
uint16_t IMU_Config = 0;
uint8_t uart_transmission_enable = 0;

uint8_t calibration_status = CHAR2_CALIBRATION_STATUS_UNINITIALIZED;
calibration_data_t calibration_data;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Queue object used for app messages
static Queue_Struct appMsgQueue;
static Queue_Handle appMsgQueueHandle;

// Clock instance for internal periodic events. Only one is needed since
// GattServApp will handle notifying all connected GATT clients
static Clock_Struct clkPeriodic;
// Clock instance for RPA read events.
static Clock_Struct clkRpaRead;

// Memory to pass periodic event ID to clock handler
spClockEventData_t argPeriodic =
{ .event = SP_PERIODIC_EVT };

// Memory to pass RPA read event ID to clock handler
spClockEventData_t argRpaRead =
{ .event = SP_READ_RPA_EVT };

// Per-handle connection info
static spConnRec_t connList[MAX_NUM_BLE_CONNS];

// List to store connection handles for set phy command status's
static List_List setPhyCommStatList;

// List to store connection handles for queued param updates
static List_List paramUpdateList;

// Auto connect Disabled/Enabled {0 - Disabled, 1- Group A , 2-Group B, ...}
uint8_t autoConnect = AUTOCONNECT_DISABLE;

// Advertising handles
static uint8 advHandleLegacy;

// Address mode
static GAP_Addr_Modes_t addrMode = DEFAULT_ADDRESS_MODE;

// Current Random Private Address
static uint8 rpa[B_ADDR_LEN] = {0};


UART_Handle uart;
UART_Params uartParams;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimplePeripheral_init( void );
static void SimplePeripheral_taskFxn(UArg a0, UArg a1);

static void BatteryInfoUpdate();
static void IMUupdate();
static void IMUreset(uint8_t newValue);
static void IMUchangeConfig(uint8_t newVal1[]);

static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg);
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg);
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData);
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg);
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId);
static void SimplePeripheral_performPeriodicTask(void);
static void SimplePeripheral_updateRPA(void);
static void SimplePeripheral_clockHandler(UArg arg);
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs,
                                        uint32_t numComparison);
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status);
static void SimplePeripheral_processPairState(spPairStateData_t *pPairState);
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData);
static void SimplePeripheral_charValueChangeCB(uint8_t paramId);
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData);
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg);
static void SimplePeripheral_initPHYRSSIArray(void);
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg);
static uint8_t SimplePeripheral_addConn(uint16_t connHandle);
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle);
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle);
static void SimplePeripheral_processParamUpdate(uint16_t connHandle);
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle);
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts);
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle);
#ifdef PTM_MODE
void simple_peripheral_handleNPIRxInterceptEvent(uint8_t *pMsg);  // Declaration
static void simple_peripheral_sendToNPI(uint8_t *buf, uint16_t len);  // Declaration
#endif // PTM_MODE

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Bond Manager Callbacks
static gapBondCBs_t SimplePeripheral_BondMgrCBs =
{
  SimplePeripheral_passcodeCb,       // Passcode callback
  SimplePeripheral_pairStateCb       // Pairing/Bonding state Callback
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t SimplePeripheral_simpleProfileCBs =
{
  SimplePeripheral_charValueChangeCB // Simple GATT Characteristic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/* Callback functions for the sensor controller */


/***********************************************************************
 * @fn scCtrlReadyCallback
 *
 * @brief Callback for messages sent by the main CPU to the sensor controller.
 *
 * @param   None.
 *
 * @return  None.
 */
void scCtrlReadyCallback(void)
{

} // scCtrlReadyCallback



/***********************************************************************
 * @fn scTaskAlertCallback
 *
 * @brief Callback for messages sent by the sensor controller to the main CPU
 *        Acknowledges the hardware interrupt and notifies app for
 *        further processing.
 *
 * @param   None.
 *
 * @return  None.
 */
void scTaskAlertCallback(void)
{
    scifClearAlertIntSource(); //clear the alert interrupt source
    //queue an instance of sensor data processing:
    SimplePeripheral_enqueueMsg(SP_SENSOR_CONTROLLER_UPDATE, NULL);
    scifAckAlertEvents(); //confirm the alert as received and processed
}
/*********************************************************************
 * @fn      simple_peripheral_spin
 *
 * @brief   Spin forever
 *
 * @param   none
 */
static void simple_peripheral_spin(void)
{
  volatile uint8_t x = 0;

  while(1)
  {
    x++;
  }
}

#ifdef PTM_MODE
/*********************************************************************
* @fn      simple_peripheral_handleNPIRxInterceptEvent
*
* @brief   Intercept an NPI RX serial message and queue for this application.
*
* @param   pMsg - a NPIMSG_msg_t containing the intercepted message.
*
* @return  none.
*/
void simple_peripheral_handleNPIRxInterceptEvent(uint8_t *pMsg)
{
 // Send Command via HCI TL
 HCI_TL_SendToStack(((NPIMSG_msg_t *)pMsg)->pBuf);

 // The data is stored as a message, free this first.
 ICall_freeMsg(((NPIMSG_msg_t *)pMsg)->pBuf);

 // Free container.
 ICall_free(pMsg);
}

/*********************************************************************
* @fn      simple_peripheral_sendToNPI
*
* @brief   Create an NPI packet and send to NPI to transmit.
*
* @param   buf - pointer HCI event or data.
*
* @param   len - length of buf in bytes.
*
* @return  none
*/
static void simple_peripheral_sendToNPI(uint8_t *buf, uint16_t len)
{
 npiPkt_t *pNpiPkt = (npiPkt_t *)ICall_allocMsg(sizeof(npiPkt_t) + len);

 if (pNpiPkt)
 {
   pNpiPkt->hdr.event = buf[0]; //Has the event status code in first byte of payload
   pNpiPkt->hdr.status = 0xFF;
   pNpiPkt->pktLen = len;
   pNpiPkt->pData  = (uint8 *)(pNpiPkt + 1);

   memcpy(pNpiPkt->pData, buf, len);

   // Send to NPI
   // Note: there is no need to free this packet.  NPI will do that itself.
   NPITask_sendToHost((uint8_t *)pNpiPkt);
 }
}
#endif // PTM_MODE



static void calibration_data_set(const uint8_t *buffer, size_t buffer_size)
{
    if (buffer_size != sizeof calibration_data) {
        while (1);
    }
    memcpy(&calibration_data, buffer, buffer_size);
    calibration_status &= ~CHAR2_CALIBRATION_STATUS_UNINITIALIZED;
    calibration_status &= ~CHAR2_CALIBRATION_STATUS_ERROR_LOAD;
    calibration_status &= ~CHAR2_CALIBRATION_STATUS_ERROR_SAVE;
}


static void calibration_data_load()
{
    uint32_t status = osal_snv_read(SNV_ID_CALIBRATION_DATA, sizeof calibration_data, (uint8 *)&calibration_data);
    if(status != SUCCESS)
    {
        calibration_status |= CHAR2_CALIBRATION_STATUS_ERROR_LOAD;
        calibration_status &= ~CHAR2_CALIBRATION_STATUS_ENABLED;
    }
    else {
        calibration_status &= ~CHAR2_CALIBRATION_STATUS_ERROR_LOAD;
        calibration_status &= ~CHAR2_CALIBRATION_STATUS_UNINITIALIZED;
    }
}

static void calibration_data_save()
{
    uint32_t status = osal_snv_write(SNV_ID_CALIBRATION_DATA, sizeof calibration_data, (uint8 *)&calibration_data);
    if(status != SUCCESS)
    {
        calibration_status |= CHAR2_CALIBRATION_STATUS_ERROR_SAVE;
    }
}

static void calibration_enable(uint8_t enable)
{
    if (!enable) {
        calibration_status &= ~CHAR2_CALIBRATION_STATUS_ENABLED;
    }
    else {
        uint8_t invalid_bitmask = CHAR2_CALIBRATION_STATUS_UNINITIALIZED | CHAR2_CALIBRATION_STATUS_ERROR_LOAD;
        if (!(calibration_status & invalid_bitmask)) {
            calibration_status |= CHAR2_CALIBRATION_STATUS_ENABLED;
        }
    }
}



/*********************************************************************
 * @fn      SimplePeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 */
void SimplePeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = spTaskStack;
  taskParams.stackSize = SP_TASK_STACK_SIZE;
  taskParams.priority = SP_TASK_PRIORITY;

  Task_construct(&spTask, SimplePeripheral_taskFxn, &taskParams, NULL);
}

void Timer_Callback_LED(Timer_Handle handle, int_fast16_t status)
{
    static uint8_t count = 0;
    if ((IMU_Config & IMU_ENABLE_SC_BM) == 0) {
        GPIO_LED_off(CONFIG_GPIO_LED_GREEN);
    }

    uint8_t battery_ticks = 1; // if level is unknown, blink for just one tick
    if (battery_level != 255) {
        battery_ticks = battery_level / 10;
    }
    // Indicate battery status using length of red LED pulse
    if (count == 0) {
        GPIO_LED_on(CONFIG_GPIO_LED_RED);
    }
    else if (count >= battery_ticks) {
        GPIO_LED_off(CONFIG_GPIO_LED_RED);
    }

    count += 1;
    if (count >= 10) {
        count = 0;
    }
}

/*********************************************************************
 * @fn      SimplePeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 */
static void SimplePeripheral_init(void)
{
  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- init ", SP_TASK_PRIORITY);
  // Create the menu

  GPIO_init();

  Timer_init(); // Init timer hardware before trying to use any timers

  Timer_Params_init(&timer_params_led);
  timer_params_led.periodUnits = Timer_PERIOD_HZ;
  timer_params_led.period = 10;
  timer_params_led.timerMode  = Timer_CONTINUOUS_CALLBACK;
  timer_params_led.timerCallback = Timer_Callback_LED;
  timer_handle_led = Timer_open(CONFIG_TIMER_LED, &timer_params_led);
  if (timer_handle_led == NULL) {
      // Timer_open() failed
      while (1);
  }
  int32_t status = Timer_start(timer_handle_led);
  if (status == Timer_STATUS_ERROR) {
      //Timer_start() failed
      while (1);
  }

  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  // Set device's Sleep Clock Accuracy
#if ( HOST_CONFIG & ( CENTRAL_CFG | PERIPHERAL_CFG ) )
  HCI_EXT_SetSCACmd(500);
#endif // (CENTRAL_CFG | PERIPHERAL_CFG)
  RCOSC_enableCalibration();
#endif // USE_RCOSC

  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueueHandle = Util_constructQueue(&appMsgQueue);

  // Create one-shot clock for internal periodic events.
  Util_constructClock(&clkPeriodic, SimplePeripheral_clockHandler,
                      SP_PERIODIC_EVT_PERIOD, 0, false, (UArg)&argPeriodic);

  // Set the Device Name characteristic in the GAP GATT Service
  // For more information, see the section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Configure GAP
  {
    uint16_t paramUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION;

    // Pass all parameter update requests to the app for it to decide
    GAP_SetParamValue(GAP_PARAM_LINK_UPDATE_DECISION, paramUpdateDecision);
  }

  // Setup the GAP Bond Manager. For more information see the GAP Bond Manager
  // section in the User's Guide
  setBondManagerParameters();

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT Service
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT Service
  DevInfo_AddService();                        // Device Information Service
  Battery_AddService();                        // Battery Service
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

  // Setup the SimpleProfile Characteristic Values
  // For more information, see the GATT and GATTServApp sections in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  {
    //uint8_t charValue1 = 1;
//    uint8_t charValue2 = 2;
//    uint8_t charValue1[SIMPLEPROFILE_CHAR1_LEN] = { 0, 0};
//    uint8_t charValue3 = 3;
//    uint8_t charValue4 = 4;
    //uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

    //SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
    //                           &charValue1);
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, SIMPLEPROFILE_CHAR1_LEN,
//                               charValue1);
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
//                               &charValue2);
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
//                               &charValue3);
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
//                               &charValue4);
   // SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               //charValue5);
  }

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimplePeripheral_simpleProfileCBs);

  // Start Bond Manager and register callback
  VOID GAPBondMgr_Register(&SimplePeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages. This is needed to receive HCI
  // events. For more information, see the HCI section in the User's Guide:
  // http://software-dl.ti.com/lprf/ble5stack-latest/
  GAP_RegisterForMsgs(selfEntity);

  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);

  // Set default values for Data Length Extension
  // Extended Data Length Feature is already enabled by default
  {
    // Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    // Some brand smartphone is essentially needing 251/2120, so we set them here.
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)

    // This API is documented in hci.h
    // See the LE Data Length Extension section in the BLE5-Stack User's Guide for information on using this command:
    // http://software-dl.ti.com/lprf/ble5stack-latest/
    HCI_LE_WriteSuggestedDefaultDataLenCmd(APP_SUGGESTED_PDU_SIZE, APP_SUGGESTED_TX_TIME);
  }

  // Initialize GATT Client
  GATT_InitClient();

  // Initialize Connection List
  SimplePeripheral_clearConnListEntry(LINKDB_CONNHANDLE_ALL);

  BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- call GAP_DeviceInit", GAP_PROFILE_PERIPHERAL);
  //Initialize GAP layer for Peripheral role and register to receive GAP events
  //GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, addrMode, &pRandomAddress);
  //changed to static MAC-address for testing
  GAP_DeviceInit(GAP_PROFILE_PERIPHERAL, selfEntity, ADDRMODE_PUBLIC, &pRandomAddress);

  // Initialize array to store connection handle and RSSI values
  SimplePeripheral_initPHYRSSIArray();

  // The type of display is configured based on the BOARD_DISPLAY_USE...
  // preprocessor definitions
  //dispHandle = Display_open(Display_Type_ANY, NULL);

#ifdef PTM_MODE
  // Intercept NPI RX events.
  NPITask_registerIncomingRXEventAppCB(simple_peripheral_handleNPIRxInterceptEvent, INTERCEPT);

  // Register for Command Status information
  HCI_TL_Init(NULL, (HCI_TL_CommandStatusCB_t) simple_peripheral_sendToNPI, NULL, selfEntity);

  // Register for Events
  HCI_TL_getCmdResponderID(ICall_getLocalMsgEntityId(ICALL_SERVICE_CLASS_BLE_MSG, selfEntity));

  // Inform Stack to Initialize PTM
  HCI_EXT_EnablePTMCmd();
#endif // PTM_MODE

  // Initialize the Sensor Controller
  scifOsalInit();
  scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
  scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
  scifInit(&scifDriverSetup);

  // Configure Sensor Controller tasks
  //scifTaskData.adcLevelTrigger.cfg.threshold = 600;

  // Start Sensor Controller task
  scifStartTasksNbl(BV(SCIF_IMUV6_SENSOR_TASK_TASK_ID));
  // Stop Sensor Controller clock to prevent sensor
  scifTaskData.imuv6SensorTask.cfg.sensorConfig = SCIF_IMUV6_SENSOR_TASK_CONFIG_ENABLE_BATTERY_BV;
  scifTaskData.imuv6SensorTask.cfg.batterySamplingInterval = 1;
  scifStartRtcTicksNow(0x00010000); // 1 Second interval to update battery statistics

  char versionBuffer[SIMPLEPROFILE_CHAR5_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  strncpy(versionBuffer, BUILD_VERSION_STRING, SIMPLEPROFILE_CHAR5_LEN);
  SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, versionBuffer);

  calibration_data_load();
  calibration_enable(1); // Try to enable calibration (will fail if load failed);

  UART_init();
  /* Create a UART with data processing off. */
      UART_Params_init(&uartParams);
      uartParams.writeDataMode = UART_DATA_BINARY;
      uartParams.readDataMode = UART_DATA_BINARY;
      uartParams.readReturnMode = UART_RETURN_FULL;
      uartParams.baudRate = 115200;



      uart = UART_open(CONFIG_DISPLAY_UART, &uartParams);



      if (uart == NULL) {
          // UART_open() failed
          while (1);
      }
}

/*********************************************************************
 * @fn      NEG_INT16
 *
 * @brief   Negate the sign of an int16_t but prevent overflow on -32768.
 *
 * @param   int16_t value.
 *
 * @return  -value, except if value == -32768, then 32676.
 */

inline static int16_t NEG_INT16(int16_t value)
{
    if (value == -32768) {
        return 32767; // largest possible positive value
    }
    return -value;
}
#define POS_INT16(value) (value) // do nothing, only markup sugar for symmetry

/*********************************************************************
 * @fn      BatteryInfoUpdate
 *
 * @brief   Processes battery data and update the battery characteristic CHAR2.
 *
 * @param   None.
 *
 * @return  None.
 */
static void BatteryInfoUpdate()
{
    if (scifTaskData.imuv6SensorTask.output.batteryStateOfCharge == 255) {
        return;
    }
    battery_level = scifTaskData.imuv6SensorTask.output.batteryStateOfCharge;
    Battery_SetParameter(battery_level);

    int16_t BatCurrent = scifTaskData.imuv6SensorTask.output.batteryCurrent;
    int16_t BatCurrentAverage = scifTaskData.imuv6SensorTask.output.batteryCurrentAverage;
    int16_t BatCapacity = scifTaskData.imuv6SensorTask.output.batteryCapacity;
    int16_t BatCapacityFull = scifTaskData.imuv6SensorTask.output.batteryCapacityFull;

    uint8_t version_major = BUILD_VERSION_STRING[1] - '0'; // Assumes version string format "vA.B.C*"
    uint8_t version_minor = BUILD_VERSION_STRING[3] - '0'; // Assumes version string format "vA.B.C*"
    int version_patch = atoi(BUILD_VERSION_STRING + 5); // Assumes version string format "vA.B.C*"
    if (version_major > 15 || version_minor > 15 || version_patch > 255) {
        while (1); // cannot represent version number with 2 bytes
    }

    char2_contents_t char2_value;
    char2_value.battery_percent = battery_level;
    char2_value.battery_voltage = TO_BE16(scifTaskData.imuv6SensorTask.output.batteryVoltage);
    char2_value.battery_current = TO_BE16(scifTaskData.imuv6SensorTask.output.batteryCurrent);
    char2_value.battery_current_average = TO_BE16(scifTaskData.imuv6SensorTask.output.batteryCurrentAverage);
    char2_value.battery_capacity = TO_BE16(scifTaskData.imuv6SensorTask.output.batteryCapacity);
    char2_value.battery_capacity_full = TO_BE16(scifTaskData.imuv6SensorTask.output.batteryCapacityFull);

    char2_value.calibration_status = calibration_status;
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof char2_value, (uint8_t*)&char2_value);

    scifTaskData.imuv6SensorTask.output.batteryStateOfCharge = 255; // Mark this data frame as processed.
}

size_t serialize_float_array_int16_BE(const float* values, size_t count, float factor, uint8_t *target_buffer)
{
    for (int i = 0; i < count; i++) {
        float v = roundf(values[i] * factor);
        v = v > INT16_MAX ? INT16_MAX : (v < -INT16_MAX ? -INT16_MAX : v);
        target_buffer[2 * i + 0] = ((int16_t) v) >> 8;
        target_buffer[2 * i + 1] = ((int16_t) v) >> 0;
    }
    return 2 * count;
}

size_t Vector3_serialize_int16_BE(const Vector3 value, float factor, uint8_t *buffer)
{
    return serialize_float_array_int16_BE(value.d, 3, factor, buffer);
}

size_t Quaternion_serialize_int16_BE(const Quaternion value, float factor, uint8_t *buffer)
{
    return serialize_float_array_int16_BE(value.d, 4, factor, buffer);
}


/*********************************************************************
 * @fn      IMUupdate
 *
 * @brief   Processes data sent by the sensor controller and update the characteristics.
 *
 * @param   None.
 *
 * @return  None.
 */

static void IMUupdate()
{
    GPIO_LED_on(CONFIG_GPIO_LED_GREEN); // Visual indication that IMU measurement is running

    //ADXL355 accelerometer coordinate transformed into universal IMU system
    Vector3 accel_adxl355_raw, accel_adxl355_calibrated;
    accel_adxl355_raw.x = POS_INT16(scifTaskData.imuv6SensorTask.output.adxl355axis[2]);  //ADXL355 accel y-axis
    accel_adxl355_raw.y = POS_INT16(scifTaskData.imuv6SensorTask.output.adxl355axis[0]);  //ADXL355 accel x-axis
    accel_adxl355_raw.z = NEG_INT16(scifTaskData.imuv6SensorTask.output.adxl355axis[4]);  //ADXL355 accel z-axis inverted

    //BMI160 accelerometer coordinate transformed into universal IMU system
    Vector3 accel_bmi160_raw, accel_bmi160_calibrated;
    accel_bmi160_raw.x = POS_INT16(scifTaskData.imuv6SensorTask.output.bmi160accelaxis[0]);  //BMI160 accel x-axis
    accel_bmi160_raw.y = NEG_INT16(scifTaskData.imuv6SensorTask.output.bmi160accelaxis[1]);  //BMI160 accel y-axis inverted
    accel_bmi160_raw.z = NEG_INT16(scifTaskData.imuv6SensorTask.output.bmi160accelaxis[2]);  //BMI160 accel z-axis inverted

    //BMI160 gyroscope coordinate transformed into universal IMU system
    Vector3 gyro_raw, gyro_radian;
    gyro_raw.x = POS_INT16(scifTaskData.imuv6SensorTask.output.bmi160gyroaxis[0]);  //BMI160 gyro x-axis
    gyro_raw.y = NEG_INT16(scifTaskData.imuv6SensorTask.output.bmi160gyroaxis[1]);  //BMI160 gyro y-axis inverted
    gyro_raw.z = NEG_INT16(scifTaskData.imuv6SensorTask.output.bmi160gyroaxis[2]);  //BMI160 gyro z-axis inverted
    //Convert gyroscope values to rad/s
    gyro_radian = Vector3_mul(gyro_raw, (float)(1.0/GYRO_SCALING_FACTOR)); // == 3.14159265358979f / 16.384f / 180.0f


    uint_fast8_t calibration_enable = (calibration_status & CHAR2_CALIBRATION_STATUS_ENABLED) != 0;
    if (calibration_enable) {
        accel_adxl355_calibrated = Vector3_mul(accel_adxl355_raw, (float)(1.0/ACCEL_ADXL355_SCALING_FACTOR)); // Scale raw sensor output to real-world units
        accel_adxl355_calibrated = imu_calibration_apply(accel_adxl355_calibrated, calibration_data.accel_misalign_scale_adxl, calibration_data.accel_bias_adxl);
        accel_bmi160_calibrated = Vector3_mul(accel_bmi160_raw, (float)(1.0/ACCEL_BMI160_SCALING_FACTOR)); // Scale raw sensor output to real-world units
        accel_bmi160_calibrated = imu_calibration_apply(accel_bmi160_calibrated, calibration_data.accel_misalign_scale_bmi, calibration_data.accel_bias_bmi);
        gyro_radian = imu_calibration_apply(gyro_radian, calibration_data.gyro_misalign_scale, calibration_data.gyro_bias);
    }

    // Rotate all sensor data 180 degrees around X-Axis (this behaves strangely, are the gravity axes left-handed after doing this?)
//    gyro_raw.x *= -1;
//    gyro_raw.z *= -1;
//    accel_adxl355_raw.x *= -1;
//    accel_adxl355_raw.z *= -1;
//    accel_bmi160_raw.x *= -1;
//    accel_bmi160_raw.z *= -1;
//    gyro_radian.x *= -1;
//    gyro_radian.z *= -1;
//    accel_adxl355_calibrated.x *= -1;
//    accel_adxl355_calibrated.z *= -1;
//    accel_bmi160_calibrated.x *= -1;
//    accel_bmi160_calibrated.z *= -1;

    //use mask to determine if Accel selection bit = 0 -> ADXL355. if = 1, then BMI180
    if ((IMU_Config & IMU_SELECT_FILTER_ACCEL_BM) == 0) {
        accel_for_filter = calibration_enable ? accel_adxl355_calibrated : accel_adxl355_raw;
    }
    else {
        accel_for_filter = calibration_enable ? accel_bmi160_calibrated : accel_bmi160_raw;
    }

    // If IMU_RESET_QUATERNION_BM is sent, integrate the accelerometer data to determine an initial quaternion
    if (reset_accel_average_counter > 0) {
        Vector3_increment(&accel_accumulator, &accel_for_filter);
        reset_accel_average_counter -= 1;
        if (reset_accel_average_counter == 0) {
            Vector3 rest_orientation_accel = {0, 0, 1};
            // Calculate a shortest-arc quaternion that represents the current direction of the accelerometer values
            getRotationBetweenVectors(&current_q, &rest_orientation_accel, &accel_accumulator);
            // The coordinate system of getRotationBetweenVectors seems to be rotated by 180 degrees around Z 
            current_q.x = -current_q.x;
            current_q.y = -current_q.y;
        }
        else {
            return; // Don't send IMU values yet, just collect data for calculating the initial quaternion.
        }
    }



    //calculate quaternion
    madgwickImuFilterUpdate(
        gyro_radian.x, gyro_radian.y, gyro_radian.z,
        accel_for_filter.x, accel_for_filter.y, accel_for_filter.z,
        &current_q.w, &current_q.x, &current_q.y, &current_q.z);

    // Repeat madgwick filter multiple times, so that the accelerometer correction converges faster.
    // Useful for debugging the axis alignments, because incorrect alignment leads to drift.
    /*
    for (int i = 0; i < 20; i++) {
        madgwickImuFilterUpdate(
            0, 0, 0,
            accel_x_f, accel_y_f, accel_z_f,
            &current_q.w, &current_q.x, &current_q.y, &current_q.z);//}
    }
    */

    //update notification depending on set transmission frequency
    if(transmission_counter == IMU_TRANSM_FREQ){

        uint8_t newVal[SIMPLEPROFILE_CHAR4_LEN];
        memset( newVal, 0, SIMPLEPROFILE_CHAR4_LEN*sizeof(uint8_t) );
        uint8_t MAX_LEN = SIMPLEPROFILE_CHAR4_LEN;
        uint8_t i = 0, len = 0;
        size_t bytes_written = 0;


        if((IMU_Config & IMU_INDEX_EN_BM) != 0){ //bit mask check
            len = 2; //length in bytes of the Index
            if(i <= (MAX_LEN - len)){ //check whether index will fit into the array
                newVal[i] =  sensor_set_index>>8; //successively add the value
                newVal[i + 1] =  sensor_set_index;
                i += len; //update the length of the index
            }
        }

        if((IMU_Config & IMU_QUATERNION_EN_BM) != 0){
            len = 8;
            if(i <= (MAX_LEN - len)){
                bytes_written = Quaternion_serialize_int16_BE(current_q, QUATERNION_SCALING_FACTOR, newVal + i);
                assert(bytes_written == len);
                i += len;
            }
        }
        if((IMU_Config & IMU_ACCEL_ADXL355_EN_BM) != 0){
            len = 6;
            if (i <= (MAX_LEN - len)){
                if (calibration_enable) {
                    bytes_written = Vector3_serialize_int16_BE(accel_adxl355_calibrated, ACCEL_ADXL355_SCALING_FACTOR, newVal + i);
                }
                else {
                    bytes_written = Vector3_serialize_int16_BE(accel_adxl355_raw, 1, newVal + i);
                }
                assert(bytes_written == len);
                i += len;
            }
        }
        if((IMU_Config & IMU_ACCEL_BMI160_EN_BM) != 0){
            len = 6;
            if (i <= (MAX_LEN - len)){
                if (calibration_enable) {
                    bytes_written = Vector3_serialize_int16_BE(accel_bmi160_calibrated, ACCEL_BMI160_SCALING_FACTOR, newVal + i);
                }
                else {
                    bytes_written = Vector3_serialize_int16_BE(accel_bmi160_raw, 1, newVal + i);
                }
                assert(bytes_written == len);
                i += len;
            }
        }
        //transmit gyroscope values
        if((IMU_Config & IMU_GYROS_EN_BM) != 0){
            len = 6;
            if(i <= (MAX_LEN - len)){
                if (calibration_enable) {
                    bytes_written = Vector3_serialize_int16_BE(gyro_radian, GYRO_SCALING_FACTOR, newVal + i);
                }
                else {
                    bytes_written = Vector3_serialize_int16_BE(gyro_raw, 1, newVal + i);
                }
                assert(bytes_written == len);
                i += len;
            }
        }
        if((IMU_Config & IMU_MAGNET_EN_BM) != 0){
            len = 6;
            if(i <= (MAX_LEN - len)){
                newVal[i] =  scifTaskData.imuv6SensorTask.output.mmc34160pjaxis[0]>>8;
                newVal[i + 1] =  scifTaskData.imuv6SensorTask.output.mmc34160pjaxis[0];
                newVal[i + 2] =  scifTaskData.imuv6SensorTask.output.mmc34160pjaxis[1]>>8;
                newVal[i + 3] =  scifTaskData.imuv6SensorTask.output.mmc34160pjaxis[1];
                newVal[i + 4] =  scifTaskData.imuv6SensorTask.output.mmc34160pjaxis[2]>>8;
                newVal[i + 5] =  scifTaskData.imuv6SensorTask.output.mmc34160pjaxis[2];
                i += len;
            }
        }
        if((IMU_Config & IMU_EXT_ANALOG_EN_BM) != 0){
            len = 1;
            if(i <= (MAX_LEN - len)){
                newVal[i] =  ((uint16_t)(scifTaskData.imuv6SensorTask.output.adcValue * 0.98))>>3; //resolution reduced to 1 byte, strange fitting to max value
                i += len;
            };
        }
        if((IMU_Config & IMU_TEMP_EN_BM) != 0){
            len = 2;
            if(i <= (MAX_LEN - len)){
                newVal[i] =  scifTaskData.imuv6SensorTask.output.temp>>8;
                newVal[i + 1] =  scifTaskData.imuv6SensorTask.output.temp;
                i += len;
            };
        }
        if((IMU_Config & IMU_PRES_EN_BM) != 0){
            len = 2;
            if(i <= (MAX_LEN - len)){
                newVal[i] =  scifTaskData.imuv6SensorTask.output.pressure>>8;
                newVal[i + 1] =  scifTaskData.imuv6SensorTask.output.pressure;
                i += len;
            };
        }
        //transmit forward error correction values for gyroscope axes, needed for calibration
        if((IMU_Config & IMU_GYRO_ECC_EN_BM) != 0){
            len = 6;
            if(i <= (MAX_LEN - len)){
                int16_t gyro_ecc_x = imu_ecc_next(0, (int16_t)gyro_raw.x);
                int16_t gyro_ecc_y = imu_ecc_next(1, (int16_t)gyro_raw.y);
                int16_t gyro_ecc_z = imu_ecc_next(2, (int16_t)gyro_raw.z);
                newVal[i + 0] =  gyro_ecc_x>>8;
                newVal[i + 1] =  gyro_ecc_x;
                newVal[i + 2] =  gyro_ecc_y>>8;
                newVal[i + 3] =  gyro_ecc_y;
                newVal[i + 4] =  gyro_ecc_z>>8;
                newVal[i + 5] =  gyro_ecc_z;
                i += len;
            }
        }

        //Add data to bt characteristic
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR4_LEN, newVal);

        transmission_counter = 0;

        //UART Transmission of index, acceleration and gyroscope values; needed for calibration; use with uart_transmission.py

        if (uart_transmission_enable)
        {
            //composition of data array to transmit via uart
            uint8_t LEN_UART = 18; //length in byte of index+accel+gyro+fletchers checksum
            uint8_t uart_newVal[LEN_UART];
            uint8_t start_newVal[5] = {83,84,65,82,84}; //ASCII-Code for "Start"
            uint8_t end_newVal[3] = {69,78,68}; //ASCII-Code for "End"
            uart_newVal[0] = sensor_set_index>>8;
            uart_newVal[1] = sensor_set_index;
            Vector3_serialize_int16_BE(accel_bmi160_raw, 1, uart_newVal + 2);
            Vector3_serialize_int16_BE(gyro_raw, 1, uart_newVal + 8);


            //Calculating Fletchers checksum and adding it to the data array
            uint16_t fsum1 = 0;
            uint16_t fsum2 = 0;
            uint8_t fi;
            for (fi = 0; fi<(LEN_UART-4); fi++){
                fsum1 = (fsum1 + uart_newVal[fi])%255;
                fsum2 = (fsum1 + fsum2)%255;
            }

            uart_newVal[14] = fsum1 >> 8;
            uart_newVal[15] = fsum1;
            uart_newVal[16] = fsum2>>8;
            uart_newVal[17] = fsum2;


            //writing data to the uart connection that is opened in static void SimplePeripheralInit(void)


            UART_write(uart, start_newVal, 5); //shows where a data package starts
            UART_write(uart, uart_newVal, LEN_UART); //actual data
            UART_write(uart, end_newVal, 3); //shows where a data package ends
        }
    }

    transmission_counter++;
    sensor_set_index++;


}
/*********************************************************************
 * @fn      IMUreset
 *
 * @brief   Resets Sensors to reset offset.  Must only be called when the IMU is in resting.
 *
 * @param   resetMode: Bit field determining what to reset:
 *              - IMU_RESET_SENSORS_BM: Re-initialize sensor ICs, including zero-calibration
 *                                      Only do this when the IMU is not moving!
 *              - IMU_RESET_QUATERNION_BM: Reset the quaternion to represent the current accelereation.
 *              - IMU_RESET_INDEX_BM: Reset the packet index to zero. Will also reset the gyro ECC state.
 *
 * @return  None.
 */
static void IMUreset(uint8_t resetMode)
{
    if(resetMode & IMU_RESET_SENSORS_BM) {
        scifTaskData.imuv6SensorTask.cfg.resetSensors = resetMode; //set reset signal for the sensor controller
    }
    if(resetMode & IMU_RESET_QUATERNION_BM) {
        reset_accel_average_counter = 10; // Wait for N sensor samples before calculating starting quaternion
        accel_accumulator = (Vector3) {{ 0, 0, 0 }}; // Reset global acceleration storage for averaging
    }
    if(resetMode & IMU_RESET_INDEX_BM) {
        sensor_set_index = 0; //reset sensor index to identify reset in data
        imu_ecc_reset(); //reset error correction state
    }
}
/*********************************************************************
 * @fn      IMUchangeConfig
 *
 * @brief   Change IMU config, like pausing sensor data readout and selecting which sensor data to readout
 *
 * @param   None.
 *
 * @return  None.
 */
static void IMUchangeConfig(uint8_t newVal1[])
{
    uint16_t old_config = IMU_Config;
    IMU_Config = (newVal1[0] << 8) | newVal1[1]; //fill variable
    // Check if sensor controller was previously disabled and must now be enabled.
    if ((old_config & IMU_ENABLE_SC_BM) == 0 && (IMU_Config & IMU_ENABLE_SC_BM) != 0) {
        IMUreset(IMU_RESET_QUATERNION_BM | IMU_RESET_INDEX_BM);
        // Set RTC to frequency (100 Hz) expected by filter algorithm
        if ((IMU_Config & IMU_ACCEL_ADXL355_EN_BM) || ((IMU_Config & IMU_QUATERNION_EN_BM) && (IMU_Config & IMU_SELECT_FILTER_ACCEL_BM))) {
            scifTaskData.imuv6SensorTask.cfg.sensorConfig |= SCIF_IMUV6_SENSOR_TASK_CONFIG_ENABLE_ADXL355_ACCEL_BV;
        }
        if ((IMU_Config & IMU_ACCEL_BMI160_EN_BM) || ((IMU_Config & IMU_QUATERNION_EN_BM) && !(IMU_Config & IMU_SELECT_FILTER_ACCEL_BM))) {
            scifTaskData.imuv6SensorTask.cfg.sensorConfig |= SCIF_IMUV6_SENSOR_TASK_CONFIG_ENABLE_BMI160_ACCEL_BV;
        }
        if ((IMU_Config & IMU_GYROS_EN_BM) || (IMU_Config & IMU_QUATERNION_EN_BM)) {
            scifTaskData.imuv6SensorTask.cfg.sensorConfig |= SCIF_IMUV6_SENSOR_TASK_CONFIG_ENABLE_BMI160_GYRO_BV;
        }
        if (IMU_Config & IMU_EXT_ANALOG_EN_BM) {
            scifTaskData.imuv6SensorTask.cfg.sensorConfig |= SCIF_IMUV6_SENSOR_TASK_CONFIG_ENABLE_ANALOG_BV;
        }
        if (IMU_Config & IMU_MAGNET_EN_BM) {
            scifTaskData.imuv6SensorTask.cfg.sensorConfig |= SCIF_IMUV6_SENSOR_TASK_CONFIG_ENABLE_MAGNETOMETER_BV;
        }
        if (IMU_Config & IMU_PRES_EN_BM) {
            scifTaskData.imuv6SensorTask.cfg.sensorConfig |= SCIF_IMUV6_SENSOR_TASK_CONFIG_ENABLE_BAROMETER_BV;
        }
        scifStartRtcTicksNow(0x00010000 / IMU_SC_SAMPLE_RATE);
        scifTaskData.imuv6SensorTask.cfg.batterySamplingInterval = IMU_SC_SAMPLE_RATE; // Sample battery values once per second
    }
    else if ((IMU_Config & IMU_ENABLE_SC_BM) == 0) {
        // TODO: Check which sensors can be disabled in scifTaskData.imuv6SensorTask.cfg.sensorConfig?
        scifStartRtcTicksNow(0x00010000); // Reset sensor controller to 1 Hz to check the battery status periodically
        scifTaskData.imuv6SensorTask.cfg.batterySamplingInterval = 1; // Sample battery values once per second
    }
    // else { } // just update the global IMU_Config variable
}
/*********************************************************************
 * @fn      SimplePeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 */
static void SimplePeripheral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
  SimplePeripheral_init();

  // Application main loop
  for (;;)
  {
    uint32_t events;

    // Waits for an event to be posted associated with the calling thread.
    // Note that an event associated with a thread is posted when a
    // message is queued to the message receive queue of the thread
    events = Event_pend(syncEvent, Event_Id_NONE, SP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      // Fetch any available messages that might have been sent from the stack
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature != 0xffff)
          {
            // Process inter-task message
            safeToDealloc = SimplePeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      if (events & SP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueueHandle))
        {
          spEvt_t *pMsg = (spEvt_t *)Util_dequeueMsg(appMsgQueueHandle);
          if (pMsg)
          {
            // Process message.
            SimplePeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  // Always dealloc pMsg unless set otherwise
  uint8_t safeToDealloc = TRUE;

  BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : Stack msg status=%d, event=0x%x\n", pMsg->status, pMsg->event);

  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimplePeripheral_processGapMessage((gapEventHdr_t*) pMsg);
      break;

    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimplePeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
    {
      // Process HCI message
      switch(pMsg->status)
      {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
        // Process HCI Command Complete Events here
        {
          SimplePeripheral_processCmdCompleteEvt((hciEvt_CmdComplete_t *) pMsg);
          break;
        }

        case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
          AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
          break;

        // HCI Commands Events
        case HCI_COMMAND_STATUS_EVENT_CODE:
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;
          switch ( pMyMsg->cmdOpcode )
          {
            case HCI_LE_SET_PHY:
            {
              if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
              {
                Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
                        "PHY Change failure, peer does not support this");
              }
              else
              {
                Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
                               "PHY Update Status Event: 0x%x",
                               pMyMsg->cmdStatus);
              }

              SimplePeripheral_updatePHYStat(HCI_LE_SET_PHY, (uint8_t *)pMsg);
              break;
            }

            default:
              break;
          }
          break;
        }

        // LE Events
        case HCI_LE_EVENT_CODE:
        {
          hciEvt_BLEPhyUpdateComplete_t *pPUC =
            (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

          // A Phy Update Has Completed or Failed
          if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
          {
            if (pPUC->status != SUCCESS)
            {
              Display_printf(dispHandle, SP_ROW_STATUS_1, 0,
                             "PHY Change failure");
            }
            else
            {
              // Only symmetrical PHY is supported.
              // rxPhy should be equal to txPhy.
              Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
                             "PHY Updated to %s",
                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_1M) ? "1M" :
                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_2M) ? "2M" :
                             (pPUC->rxPhy == PHY_UPDATE_COMPLETE_EVENT_CODED) ? "CODED" : "Unexpected PHY Value");
            }

            SimplePeripheral_updatePHYStat(HCI_BLE_PHY_UPDATE_COMPLETE_EVENT, (uint8_t *)pMsg);
          }
          break;
        }

        default:
          break;
      }

      break;
    }

    default:
      // do nothing
      break;
  }

#ifdef PTM_MODE
  // Check for NPI Messages
  hciPacket_t *pBuf = (hciPacket_t *)pMsg;

  // Serialized HCI Event
  if (pBuf->hdr.event == HCI_CTRL_TO_HOST_EVENT)
  {
    uint16_t len = 0;

    // Determine the packet length
    switch(pBuf->pData[0])
    {
      case HCI_EVENT_PACKET:
        len = HCI_EVENT_MIN_LENGTH + pBuf->pData[2];
        break;

      case HCI_ACL_DATA_PACKET:
        len = HCI_DATA_MIN_LENGTH + BUILD_UINT16(pBuf->pData[3], pBuf->pData[4]);
        break;

      default:
        break;
    }

    // Send to Remote Host.
    simple_peripheral_sendToNPI(pBuf->pData, len);

    // Free buffers if needed.
    switch (pBuf->pData[0])
    {
      case HCI_ACL_DATA_PACKET:
      case HCI_SCO_DATA_PACKET:
        BM_free(pBuf->pData);
      default:
        break;
    }
  }
#endif // PTM_MODE

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimplePeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimplePeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
      UART_Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
      UART_Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimplePeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimplePeripheral_processAppMsg(spEvt_t *pMsg) //work through task schedule
{
  bool dealloc = TRUE;

  if (pMsg->event <= APP_EVT_EVENT_MAX)
  {
    BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=%s\n", 0, appEventStrings[pMsg->event]);
  }
  else
  {
    BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : App msg status=%d, event=0x%x\n", 0, pMsg->event);
  }

  switch (pMsg->event) //do x based on the queued message
  {
    case SP_CHAR_CHANGE_EVT:
      SimplePeripheral_processCharValueChangeEvt(*(uint8_t*)(pMsg->pData));
      break;

    case SP_ADV_EVT:
      SimplePeripheral_processAdvEvent((spGapAdvEventData_t*)(pMsg->pData));
      break;

    case SP_PAIR_STATE_EVT:
      SimplePeripheral_processPairState((spPairStateData_t*)(pMsg->pData));
      break;

    case SP_PASSCODE_EVT:
      SimplePeripheral_processPasscode((spPasscodeData_t*)(pMsg->pData));
      break;

    case SP_PERIODIC_EVT:
      SimplePeripheral_performPeriodicTask();
      break;

    case SP_READ_RPA_EVT:
      SimplePeripheral_updateRPA();
      break;

    case SP_SEND_PARAM_UPDATE_EVT:
    {
      // Extract connection handle from data
      uint16_t connHandle = *(uint16_t *)(((spClockEventData_t *)pMsg->pData)->data);

      SimplePeripheral_processParamUpdate(connHandle);

      // This data is not dynamically allocated
      dealloc = FALSE;
      break;
    }

    case SP_SENSOR_CONTROLLER_UPDATE:
        if (IMU_Config & IMU_ENABLE_SC_BM) { // process incoming IMU data from sensor controller, if enabled:
            IMUupdate();
        }
        BatteryInfoUpdate(); // check if new battery data has arrived
        break;

    default:
      // Do nothing.
      break;
  }

  // Free message data if it exists and we are to dealloc
  if ((dealloc == TRUE) && (pMsg->pData != NULL))
  {
    ICall_free(pMsg->pData);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processGapMessage
 *
 * @brief   Process an incoming GAP event.
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_processGapMessage(gapEventHdr_t *pMsg)
{
  switch(pMsg->opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:
    {
      bStatus_t status = FAILURE;

      gapDeviceInitDoneEvent_t *pPkt = (gapDeviceInitDoneEvent_t *)pMsg;

      if(pPkt->hdr.status == SUCCESS)
      {
        // Store the system ID
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = pPkt->devAddr[0];
        systemId[1] = pPkt->devAddr[1];
        systemId[2] = pPkt->devAddr[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = pPkt->devAddr[5];
        systemId[6] = pPkt->devAddr[4];
        systemId[5] = pPkt->devAddr[3];

        // Set Device Info Service Parameter
        status = DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        char buffer[DEVINFO_STR_ATTR_LEN + 1];
        int len = 0;

        // Set serial number characteristic == BLE MAC address
        len = snprintf(buffer, DEVINFO_STR_ATTR_LEN, "%02X:%02X:%02X:%02X:%02X:%02X",
                 pPkt->devAddr[5], pPkt->devAddr[4], pPkt->devAddr[3],
                 pPkt->devAddr[2], pPkt->devAddr[1], pPkt->devAddr[0]);
        status = DevInfo_SetParameter(DEVINFO_SERIAL_NUMBER, len, buffer);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set firmware revision characteristic
        len = strlen(BUILD_VERSION_STRING);
        if (len > DEVINFO_STR_ATTR_LEN) {
            len = DEVINFO_STR_ATTR_LEN;
        }
        status = DevInfo_SetParameter(DEVINFO_FIRMWARE_REV, len, BUILD_VERSION_STRING);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set hardware revision characteristic
        len = get_hardware_revision_str(buffer, DEVINFO_STR_ATTR_LEN);
        status = DevInfo_SetParameter(DEVINFO_HARDWARE_REV, len, buffer);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Initialized");

        BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_DEVICE_INIT_DONE_EVENT", 0);
        // Setup and start Advertising
        // For more information, see the GAP section in the User's Guide:
        // http://software-dl.ti.com/lprf/ble5stack-latest/

        BLE_LOG_INT_INT(0, BLE_LOG_MODULE_APP, "APP : ---- call GapAdv_create set=%d,%d\n", 0, 0);
        // Create Advertisement set #1 and assign handle
        status = GapAdv_create(&SimplePeripheral_advCallback, &advParams1,
                               &advHandleLegacy);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load advertising data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_ADV,
                                     sizeof(advData1), advData1);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Load scan response data for set #1 that is statically allocated by the app
        status = GapAdv_loadByHandle(advHandleLegacy, GAP_ADV_DATA_TYPE_SCAN_RSP,
                                     sizeof(scanResData1), scanResData1);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Set event mask for set #1
        status = GapAdv_setEventMask(advHandleLegacy,
                                     GAP_ADV_EVT_MASK_START_AFTER_ENABLE |
                                     GAP_ADV_EVT_MASK_END_AFTER_DISABLE |
                                     GAP_ADV_EVT_MASK_SET_TERMINATED);

        // Enable legacy advertising for set #1
        status = GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
        SIMPLEPERIPHERAL_ASSERT(status == SUCCESS);

        // Display device address
        Display_printf(dispHandle, SP_ROW_IDA, 0, "%s Addr: %s",
                       (addrMode <= ADDRMODE_RANDOM) ? "Dev" : "ID",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        if (addrMode > ADDRMODE_RANDOM)
        {
          SimplePeripheral_updateRPA();

          // Create one-shot clock for RPA check event.
          Util_constructClock(&clkRpaRead, SimplePeripheral_clockHandler,
                              READ_RPA_PERIOD, 0, true,
                              (UArg) &argRpaRead);
        }
      }

      break;
    }

    case GAP_LINK_ESTABLISHED_EVENT:
    {
      gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsg;

      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- got GAP_LINK_ESTABLISHED_EVENT", 0);
      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      if (pPkt->hdr.status == SUCCESS)
      {
        // Add connection to list and start RSSI
        SimplePeripheral_addConn(pPkt->connectionHandle);

        // Display the address of this connection
        Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connected to %s",
                       Util_convertBdAddr2Str(pPkt->devAddr));

        // Start Periodic Clock.
        Util_startClock(&clkPeriodic);
      }
      if ((numActive < MAX_NUM_BLE_CONNS) && (autoConnect == AUTOCONNECT_DISABLE))
      {
        // Start advertising since there is room for more connections
        GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);
      }
      else
      {
        // Stop advertising since there is no room for more connections
        GapAdv_disable(advHandleLegacy);
      }
      break;
    }

    case GAP_LINK_TERMINATED_EVENT:
    {
      gapTerminateLinkEvent_t *pPkt = (gapTerminateLinkEvent_t *)pMsg;

      // Display the amount of current connections
      uint8_t numActive = linkDB_NumActive();
      Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Device Disconnected!");
      Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Num Conns: %d",
                     (uint16_t)numActive);

      // Remove the connection from the list and disable RSSI if needed
      SimplePeripheral_removeConn(pPkt->connectionHandle);

      // If no active connections
      if (numActive == 0)
      {
        // Stop periodic clock
        Util_stopClock(&clkPeriodic);
        // TODO: Stop IMU activity (Sensor controller) when all connections are gone.
      }

      BLE_LOG_INT_STR(0, BLE_LOG_MODULE_APP, "APP : GAP msg: status=%d, opcode=%s\n", 0, "GAP_LINK_TERMINATED_EVENT");
      // Start advertising since there is room for more connections
      GapAdv_enable(advHandleLegacy, GAP_ADV_ENABLE_OPTIONS_USE_MAX , 0);

      // Clear remaining lines
      Display_clearLine(dispHandle, SP_ROW_CONNECTION);

      break;
    }

    case GAP_UPDATE_LINK_PARAM_REQ_EVENT:
    {
      gapUpdateLinkParamReqReply_t rsp;

      gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsg;

      rsp.connectionHandle = pReq->req.connectionHandle;
      rsp.signalIdentifier = pReq->req.signalIdentifier;

      // Only accept connection intervals with slave latency of 0
      // This is just an example of how the application can send a response
      if(pReq->req.connLatency == 0)
      {
        rsp.intervalMin = pReq->req.intervalMin;
        rsp.intervalMax = pReq->req.intervalMax;
        rsp.connLatency = pReq->req.connLatency;
        rsp.connTimeout = pReq->req.connTimeout;
        rsp.accepted = TRUE;
      }
      else
      {
        rsp.accepted = FALSE;
      }

      // Send Reply
      VOID GAP_UpdateLinkParamReqReply(&rsp);

      break;
    }

    case GAP_LINK_PARAM_UPDATE_EVENT:
    {
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDBInfo_t linkInfo;
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      if(pPkt->status == SUCCESS)
      {
        // Display the address of the connection update
        Display_printf(dispHandle, SP_ROW_STATUS_2, 0, "Link Param Updated: %s",
                       Util_convertBdAddr2Str(linkInfo.addr));
      }
      else
      {
        // Display the address of the connection update failure
        Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
                       "Link Param Update Failed 0x%x: %s", pPkt->opcode,
                       Util_convertBdAddr2Str(linkInfo.addr));
      }

      // Check if there are any queued parameter updates
      spConnHandleEntry_t *connHandleEntry = (spConnHandleEntry_t *)List_get(&paramUpdateList);
      if (connHandleEntry != NULL)
      {
        // Attempt to send queued update now
        SimplePeripheral_processParamUpdate(connHandleEntry->connHandle);

        // Free list element
        ICall_free(connHandleEntry);
      }

      break;
    }

#if defined ( NOTIFY_PARAM_UPDATE_RJCT )
    case GAP_LINK_PARAM_UPDATE_REJECT_EVENT:
    {
      linkDBInfo_t linkInfo;
      gapLinkUpdateEvent_t *pPkt = (gapLinkUpdateEvent_t *)pMsg;

      // Get the address from the connection handle
      linkDB_GetInfo(pPkt->connectionHandle, &linkInfo);

      // Display the address of the connection update failure
      Display_printf(dispHandle, SP_ROW_STATUS_2, 0,
                     "Peer Device's Update Request Rejected 0x%x: %s", pPkt->opcode,
                     Util_convertBdAddr2Str(linkInfo.addr));

      break;
    }
#endif

    default:
      Display_clearLines(dispHandle, SP_ROW_STATUS_1, SP_ROW_STATUS_2);
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramId - parameter Id of the value that was changed.
 *
 * @return  None.
 */
static void SimplePeripheral_charValueChangeCB(uint8_t paramId)
{
  uint8_t *pValue = ICall_malloc(sizeof(uint8_t));

  if (pValue)
  {
    *pValue = paramId;

    if (SimplePeripheral_enqueueMsg(SP_CHAR_CHANGE_EVT, pValue) != SUCCESS)
    {
      ICall_free(pValue);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 */
static void SimplePeripheral_processCharValueChangeEvt(uint8_t paramId)
{
  uint8_t newVal1[2];
  uint8_t command = 0;

  switch(paramId)
  {
    case SIMPLEPROFILE_CHAR1:
        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newVal1, SIMPLEPROFILE_CHAR1_LEN);
        IMUchangeConfig(newVal1);
      break;

    case SIMPLEPROFILE_CHAR3:
        // CHAR3 is used for IMU reset and for IMU configuration.
        // We use the first byte to determine what to do.
        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, newVal1, 2);
        command = newVal1[0];
        if (command <= IMU_RESET_MAX_BIT_VALUE) // This is a reset command, handle it
        {
            IMUreset(command);
        }
        else // This is a config command, need to get the rest of the data first
        {
            uint8_t length = newVal1[1];
            uint8_t char3_buffer[SIMPLEPROFILE_CHAR3_LEN];
            if (length > SIMPLEPROFILE_CHAR3_LEN) {
                length = SIMPLEPROFILE_CHAR3_LEN;
            }
            SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, char3_buffer, length);
            switch (command) {
            case IMU_CONFIG_CALIBRATION_COEFFICIENTS:
                calibration_data_set(char3_buffer + CHAR3_HEADER_LENGTH, length - CHAR3_HEADER_LENGTH);
                break;
            case IMU_CONFIG_SAVE_CALIBRATION:
                calibration_data_save();
                break;
            case IMU_CONFIG_LOAD_CALIBRATION:
                calibration_data_load();
                break;
            case IMU_CONFIG_USE_CALIBRATION:
                calibration_enable(char3_buffer[CHAR3_HEADER_LENGTH] != 0);
                break;
            case IMU_CONFIG_ENABLE_UART_TRANSMISSION:
                uart_transmission_enable = char3_buffer[CHAR3_HEADER_LENGTH] != 0;
                break;
            }
            //if (status != SUCCESS) {
                //while(1); // unrecoverable error
            //}
        }
        //reset characteristic to zero, so that it can be triggered again
        //newValue = 0;
        //SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, 1, &newValue);
      break;

    default:
      // should not reach here!
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_performPeriodicTask(void)
{
//  uint8_t valueToCopy;
//
//  // Call to retrieve the value of the third characteristic in the profile
//  if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
//  {
//    // Call to set that value of the fourth characteristic in the profile.
//    // Note that if notifications of the fourth characteristic have been
//    // enabled by a GATT client device, then a notification will be sent
//    // every time this function is called.
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
//                               &valueToCopy);
//  }
}

/*********************************************************************
 * @fn      SimplePeripheral_updateRPA
 *
 * @brief   Read the current RPA from the stack and update display
 *          if the RPA has changed.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimplePeripheral_updateRPA(void)
{
  uint8_t* pRpaNew;

  // Read the current RPA.
  pRpaNew = GAP_GetDevAddress(FALSE);

  if (memcmp(pRpaNew, rpa, B_ADDR_LEN))
  {
    // If the RPA has changed, update the display
    Display_printf(dispHandle, SP_ROW_RPA, 0, "RP Addr: %s",
                   Util_convertBdAddr2Str(pRpaNew));
    memcpy(rpa, pRpaNew, B_ADDR_LEN);
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimplePeripheral_clockHandler(UArg arg)
{
  spClockEventData_t *pData = (spClockEventData_t *)arg;

 if (pData->event == SP_PERIODIC_EVT)
 {
   // Start the next period
   Util_startClock(&clkPeriodic);

   // Post event to wake up the application
   SimplePeripheral_enqueueMsg(SP_PERIODIC_EVT, NULL);
 }
 else if (pData->event == SP_READ_RPA_EVT)
 {
   // Start the next period
   Util_startClock(&clkRpaRead);

   // Post event to read the current RPA
   SimplePeripheral_enqueueMsg(SP_READ_RPA_EVT, NULL);
 }
 else if (pData->event == SP_SEND_PARAM_UPDATE_EVT)
 {
    // Send message to app
    SimplePeripheral_enqueueMsg(SP_SEND_PARAM_UPDATE_EVT, pData);
 }
}

/*********************************************************************
 * @fn      SimplePeripheral_advCallback
 *
 * @brief   GapAdv module callback
 *
 * @param   pMsg - message to process
 */
static void SimplePeripheral_advCallback(uint32_t event, void *pBuf, uintptr_t arg)
{
  spGapAdvEventData_t *pData = ICall_malloc(sizeof(spGapAdvEventData_t));

  if (pData)
  {
    pData->event = event;
    pData->pBuf = pBuf;

    if(SimplePeripheral_enqueueMsg(SP_ADV_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processAdvEvent
 *
 * @brief   Process advertising event in app context
 *
 * @param   pEventData
 */
static void SimplePeripheral_processAdvEvent(spGapAdvEventData_t *pEventData)
{
  switch (pEventData->event)
  {
    case GAP_EVT_ADV_START_AFTER_ENABLE:
      BLE_LOG_INT_TIME(0, BLE_LOG_MODULE_APP, "APP : ---- GAP_EVT_ADV_START_AFTER_ENABLE", 0);
      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d Enabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_END_AFTER_DISABLE:
      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d Disabled",
                     *(uint8_t *)(pEventData->pBuf));
      break;

    case GAP_EVT_ADV_START:
      break;

    case GAP_EVT_ADV_END:
      break;

    case GAP_EVT_ADV_SET_TERMINATED:
    {
//#ifndef Display_DISABLE_ALL
//      GapAdv_setTerm_t *advSetTerm = (GapAdv_setTerm_t *)(pEventData->pBuf);
//
//      Display_printf(dispHandle, SP_ROW_ADVSTATE, 0, "Adv Set %d disabled after conn %d",
//                     advSetTerm->handle, advSetTerm->connHandle );
//#endif
    }
    break;

    case GAP_EVT_SCAN_REQ_RECEIVED:
      break;

    case GAP_EVT_INSUFFICIENT_MEMORY:
      break;

    default:
      break;
  }

  // All events have associated memory to free except the insufficient memory
  // event
  if (pEventData->event != GAP_EVT_INSUFFICIENT_MEMORY)
  {
    ICall_free(pEventData->pBuf);
  }
}


/*********************************************************************
 * @fn      SimplePeripheral_pairStateCb
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimplePeripheral_pairStateCb(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  spPairStateData_t *pData = ICall_malloc(sizeof(spPairStateData_t));

  // Allocate space for the event data.
  if (pData)
  {
    pData->state = state;
    pData->connHandle = connHandle;
    pData->status = status;

    // Queue the event.
    if(SimplePeripheral_enqueueMsg(SP_PAIR_STATE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_passcodeCb
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimplePeripheral_passcodeCb(uint8_t *pDeviceAddr,
                                        uint16_t connHandle,
                                        uint8_t uiInputs,
                                        uint8_t uiOutputs,
                                        uint32_t numComparison)
{
  spPasscodeData_t *pData = ICall_malloc(sizeof(spPasscodeData_t));

  // Allocate space for the passcode event.
  if (pData )
  {
    pData->connHandle = connHandle;
    memcpy(pData->deviceAddr, pDeviceAddr, B_ADDR_LEN);
    pData->uiInputs = uiInputs;
    pData->uiOutputs = uiOutputs;
    pData->numComparison = numComparison;

    // Enqueue the event.
    if(SimplePeripheral_enqueueMsg(SP_PASSCODE_EVT, pData) != SUCCESS)
    {
      ICall_free(pData);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimplePeripheral_processPairState(spPairStateData_t *pPairData)
{
  uint8_t state = pPairData->state;
  uint8_t status = pPairData->status;

  switch (state)
  {
    case GAPBOND_PAIRING_STATE_STARTED:
      Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing started");
      break;

    case GAPBOND_PAIRING_STATE_COMPLETE:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing success");
      }
      else
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Pairing fail: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_ENCRYPTED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Encryption success");
      }
      else
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Encryption failed: %d", status);
      }
      break;

    case GAPBOND_PAIRING_STATE_BOND_SAVED:
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Bond save success");
      }
      else
      {
        Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Bond save failed: %d", status);
      }
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimplePeripheral_processPasscode(spPasscodeData_t *pPasscodeData)
{
  // Display passcode to user
  if (pPasscodeData->uiOutputs != 0)
  {
    Display_printf(dispHandle, SP_ROW_CONNECTION, 0, "Passcode: %d",
                   B_APP_DEFAULT_PASSCODE);
  }

  // Send passcode response
  GAPBondMgr_PasscodeRsp(pPasscodeData->connHandle , SUCCESS,
                         B_APP_DEFAULT_PASSCODE);
}


/*********************************************************************
 * @fn      SimplePeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 */
static status_t SimplePeripheral_enqueueMsg(uint8_t event, void *pData)
{
  uint8_t success;
  spEvt_t *pMsg = ICall_malloc(sizeof(spEvt_t));

  // Create dynamic pointer to message.
  if(pMsg)
  {
    pMsg->event = event;
    pMsg->pData = pData;

    // Enqueue the message.
    success = Util_enqueueMsg(appMsgQueueHandle, syncEvent, (uint8_t *)pMsg);
    return (success) ? SUCCESS : FAILURE;
  }

  return(bleMemAllocError);
}


/*********************************************************************
 * @fn      SimplePeripheral_addConn
 *
 * @brief   Add a device to the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is put in.
 *          if there is no room, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_addConn(uint16_t connHandle)
{
  uint8_t i;
  uint8_t status = bleNoResources;

  // Try to find an available entry
  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == LINKDB_CONNHANDLE_INVALID)
    {
      // Found available entry to put a new connection info in
      connList[i].connHandle = connHandle;

#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
      // Allocate data to send through clock handler
      connList[i].pParamUpdateEventData = ICall_malloc(sizeof(spClockEventData_t) +
                                                       sizeof (uint16_t));
      if(connList[i].pParamUpdateEventData)
      {
        connList[i].pParamUpdateEventData->event = SP_SEND_PARAM_UPDATE_EVT;
        *((uint16_t *)connList[i].pParamUpdateEventData->data) = connHandle;

        // Create a clock object and start
        connList[i].pUpdateClock
          = (Clock_Struct*) ICall_malloc(sizeof(Clock_Struct));

        if (connList[i].pUpdateClock)
        {
          Util_constructClock(connList[i].pUpdateClock,
                              SimplePeripheral_clockHandler,
                              SEND_PARAM_UPDATE_DELAY, 0, true,
                              (UArg) (connList[i].pParamUpdateEventData));
        }
        else
        {
            ICall_free(connList[i].pParamUpdateEventData);
        }
      }
      else
      {
        status = bleMemAllocError;
      }
#endif

      // Set default PHY to 1M
      connList[i].currPhy = HCI_PHY_1_MBPS;

      break;
    }
  }

  return status;
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  the index of the entry that has the given connection handle.
 *          if there is no match, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_getConnIndex(uint16_t connHandle)
{
  uint8_t i;

  for (i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if (connList[i].connHandle == connHandle)
    {
      return i;
    }
  }

  return(MAX_NUM_BLE_CONNS);
}

/*********************************************************************
 * @fn      SimplePeripheral_getConnIndex
 *
 * @brief   Find index in the connected device list by connHandle
 *
 * @return  SUCCESS if connHandle found valid index or bleInvalidRange
 *          if index wasn't found. LINKDB_CONNHANDLE_ALL will always succeed.
 */
static uint8_t SimplePeripheral_clearConnListEntry(uint16_t connHandle)
{
  uint8_t i;
  // Set to invalid connection index initially
  uint8_t connIndex = MAX_NUM_BLE_CONNS;

  if(connHandle != LINKDB_CONNHANDLE_ALL)
  {
    // Get connection index from handle
    connIndex = SimplePeripheral_getConnIndex(connHandle);
    if(connIndex >= MAX_NUM_BLE_CONNS)
	{
	  return(bleInvalidRange);
	}
  }

  // Clear specific handle or all handles
  for(i = 0; i < MAX_NUM_BLE_CONNS; i++)
  {
    if((connIndex == i) || (connHandle == LINKDB_CONNHANDLE_ALL))
    {
      connList[i].connHandle = LINKDB_CONNHANDLE_INVALID;
      connList[i].currPhy = 0;
      connList[i].phyCngRq = 0;
      connList[i].phyRqFailCnt = 0;
      connList[i].rqPhy = 0;
      memset(connList[i].rssiArr, 0, SP_MAX_RSSI_STORE_DEPTH);
      connList[i].rssiAvg = 0;
      connList[i].rssiCntr = 0;
      connList[i].isAutoPHYEnable = FALSE;
    }
  }

  return(SUCCESS);
}

/*********************************************************************
 * @fn      SimplePeripheral_clearPendingParamUpdate
 *
 * @brief   clean pending param update request in the paramUpdateList list
 *
 * @param   connHandle - connection handle to clean
 *
 * @return  none
 */
void SimplePeripheral_clearPendingParamUpdate(uint16_t connHandle)
{
  List_Elem *curr;

  for (curr = List_head(&paramUpdateList); curr != NULL; curr = List_next(curr))
  {
    if (((spConnHandleEntry_t *)curr)->connHandle == connHandle)
    {
      List_remove(&paramUpdateList, curr);
    }
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_removeConn
 *
 * @brief   Remove a device from the connected device list
 *
 * @return  index of the connected device list entry where the new connection
 *          info is removed from.
 *          if connHandle is not found, MAX_NUM_BLE_CONNS will be returned.
 */
static uint8_t SimplePeripheral_removeConn(uint16_t connHandle)
{
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);

  if(connIndex != MAX_NUM_BLE_CONNS)
  {
    Clock_Struct* pUpdateClock = connList[connIndex].pUpdateClock;

    if (pUpdateClock != NULL)
    {
      // Stop and destruct the RTOS clock if it's still alive
      if (Util_isActive(pUpdateClock))
      {
        Util_stopClock(pUpdateClock);
      }

      // Destruct the clock object
      Clock_destruct(pUpdateClock);
      // Free clock struct
      ICall_free(pUpdateClock);
      // Free ParamUpdateEventData
      ICall_free(connList[connIndex].pParamUpdateEventData);
    }
    // Clear pending update requests from paramUpdateList
    SimplePeripheral_clearPendingParamUpdate(connHandle);
    // Stop Auto PHY Change
    SimplePeripheral_stopAutoPhyChange(connHandle);
    // Clear Connection List Entry
    SimplePeripheral_clearConnListEntry(connHandle);
  }

  return connIndex;
}

/*********************************************************************
 * @fn      SimplePeripheral_processParamUpdate
 *
 * @brief   Process a parameters update request
 *
 * @return  None
 */
static void SimplePeripheral_processParamUpdate(uint16_t connHandle)
{
  gapUpdateLinkParamReq_t req;
  uint8_t connIndex;

  req.connectionHandle = connHandle;
#ifdef DEFAULT_SEND_PARAM_UPDATE_REQ
  req.connLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
  req.connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;
  req.intervalMin = 6;  //*1,25ms = 7,5ms absolute minimum
  req.intervalMax = 10; //*1,25ms = 4000ms absolute maximum
#endif

  connIndex = SimplePeripheral_getConnIndex(connHandle);
  if (connIndex >= MAX_NUM_BLE_CONNS)
  {
    Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
    return;
  }

  // Deconstruct the clock object
  Clock_destruct(connList[connIndex].pUpdateClock);
  // Free clock struct, only in case it is not NULL
  if (connList[connIndex].pUpdateClock != NULL)
  {
      ICall_free(connList[connIndex].pUpdateClock);
      connList[connIndex].pUpdateClock = NULL;
  }
  // Free ParamUpdateEventData, only in case it is not NULL
  if (connList[connIndex].pParamUpdateEventData != NULL)
      ICall_free(connList[connIndex].pParamUpdateEventData);

  // Send parameter update
  bStatus_t status = GAP_UpdateLinkParamReq(&req);

  // If there is an ongoing update, queue this for when the udpate completes
  if (status == bleAlreadyInRequestedMode)
  {
    spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));
    if (connHandleEntry)
    {
      connHandleEntry->connHandle = connHandle;

      List_put(&paramUpdateList, (List_Elem *)connHandleEntry);
    }
  }
}

/*********************************************************************
 * @fn      SimpleCentral_processCmdCompleteEvt
 *
 * @brief   Process an incoming OSAL HCI Command Complete Event.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimplePeripheral_processCmdCompleteEvt(hciEvt_CmdComplete_t *pMsg)
{
  uint8_t status = pMsg->pReturnParam[0];

  //Find which command this command complete is for
  switch (pMsg->cmdOpcode)
  {
    case HCI_READ_RSSI:
    {
      int8 rssi = (int8)pMsg->pReturnParam[3];

      // Display RSSI value, if RSSI is higher than threshold, change to faster PHY
      if (status == SUCCESS)
      {
        uint16_t handle = BUILD_UINT16(pMsg->pReturnParam[1], pMsg->pReturnParam[2]);

        uint8_t index = SimplePeripheral_getConnIndex(handle);
        if (index >= MAX_NUM_BLE_CONNS)
        {
          Display_printf(dispHandle, SP_ROW_STATUS_1, 0, "Connection handle is not in the connList !!!");
          return;
        }

        if (rssi != LL_RSSI_NOT_AVAILABLE)
        {
          connList[index].rssiArr[connList[index].rssiCntr++] = rssi;
          connList[index].rssiCntr %= SP_MAX_RSSI_STORE_DEPTH;

          int16_t sum_rssi = 0;
          for(uint8_t cnt=0; cnt<SP_MAX_RSSI_STORE_DEPTH; cnt++)
          {
            sum_rssi += connList[index].rssiArr[cnt];
          }
          connList[index].rssiAvg = (uint32_t)(sum_rssi/SP_MAX_RSSI_STORE_DEPTH);

          uint8_t phyRq = SP_PHY_NONE;
          uint8_t phyRqS = SP_PHY_NONE;
          uint8_t phyOpt = LL_PHY_OPT_NONE;

          if(connList[index].phyCngRq == FALSE)
          {
            if((connList[index].rssiAvg >= RSSI_2M_THRSHLD) &&
            (connList[index].currPhy != HCI_PHY_2_MBPS) &&
                 (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to higher data rate
              phyRqS = phyRq = HCI_PHY_2_MBPS;
            }
            else if((connList[index].rssiAvg < RSSI_2M_THRSHLD) &&
                    (connList[index].rssiAvg >= RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != HCI_PHY_1_MBPS) &&
                    (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to legacy regular data rate
              phyRqS = phyRq = HCI_PHY_1_MBPS;
            }
            else if((connList[index].rssiAvg >= RSSI_S2_THRSHLD) &&
                    (connList[index].rssiAvg < RSSI_1M_THRSHLD) &&
                    (connList[index].currPhy != SP_PHY_NONE))
            {
              // try to go to lower data rate S=2(500kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S2;
              phyRq = BLE5_CODED_S2_PHY;
            }
            else if(connList[index].rssiAvg < RSSI_S2_THRSHLD )
            {
              // try to go to lowest data rate S=8(125kb/s)
              phyRqS = HCI_PHY_CODED;
              phyOpt = LL_PHY_OPT_S8;
              phyRq = BLE5_CODED_S8_PHY;
            }
            if((phyRq != SP_PHY_NONE) &&
               // First check if the request for this phy change is already not honored then don't request for change
               (((connList[index].rqPhy == phyRq) &&
                 (connList[index].phyRqFailCnt < 2)) ||
                 (connList[index].rqPhy != phyRq)))
            {
              //Initiate PHY change based on RSSI
              SimplePeripheral_setPhy(connList[index].connHandle, 0,
                                      phyRqS, phyRqS, phyOpt);
              connList[index].phyCngRq = TRUE;

              // If it a request for different phy than failed request, reset the count
              if(connList[index].rqPhy != phyRq)
              {
                // then reset the request phy counter and requested phy
                connList[index].phyRqFailCnt = 0;
              }

              if(phyOpt == LL_PHY_OPT_NONE)
              {
                connList[index].rqPhy = phyRq;
              }
              else if(phyOpt == LL_PHY_OPT_S2)
              {
                connList[index].rqPhy = BLE5_CODED_S2_PHY;
              }
              else
              {
                connList[index].rqPhy = BLE5_CODED_S8_PHY;
              }

            } // end of if ((phyRq != SP_PHY_NONE) && ...
          } // end of if (connList[index].phyCngRq == FALSE)
        } // end of if (rssi != LL_RSSI_NOT_AVAILABLE)

        Display_printf(dispHandle, SP_ROW_RSSI, 0,
                       "RSSI:%d dBm, AVG RSSI:%d dBm",
                       (uint32_t)(rssi),
                       connList[index].rssiAvg);

	  } // end of if (status == SUCCESS)
      break;
    }

    case HCI_LE_READ_PHY:
    {
      if (status == SUCCESS)
      {
        Display_printf(dispHandle, SP_ROW_RSSI + 2, 0, "RXPh: %d, TXPh: %d",
                       pMsg->pReturnParam[3], pMsg->pReturnParam[4]);
      }
      break;
    }

    default:
      break;
  } // end of switch (pMsg->cmdOpcode)
}

/*********************************************************************
* @fn      SimplePeripheral_initPHYRSSIArray
*
* @brief   Initializes the array of structure/s to store data related
*          RSSI based auto PHy change
*
* @param   connHandle - the connection handle
*
* @param   addr - pointer to device address
*
* @return  index of connection handle
*/
static void SimplePeripheral_initPHYRSSIArray(void)
{
  //Initialize array to store connection handle and RSSI values
  memset(connList, 0, sizeof(connList));
  for (uint8_t index = 0; index < MAX_NUM_BLE_CONNS; index++)
  {
    connList[index].connHandle = SP_INVALID_HANDLE;
  }
}

/*********************************************************************
 * @fn      SimplePeripheral_stopAutoPhyChange
 *
 * @brief   Cancel periodic RSSI reads on a link.
 *
 * @param   connHandle - connection handle of link
 *
 * @return  SUCCESS: Operation successful
 *          bleIncorrectMode: No link
 */
static status_t SimplePeripheral_stopAutoPhyChange(uint16_t connHandle)
{
  // Get connection index from handle
  uint8_t connIndex = SimplePeripheral_getConnIndex(connHandle);
  SIMPLEPERIPHERAL_ASSERT(connIndex < MAX_NUM_BLE_CONNS);

  // Stop connection event notice
  Gap_RegisterConnEventCb(NULL, GAP_CB_UNREGISTER, connHandle);

  // Also update the phychange request status for active RSSI tracking connection
  connList[connIndex].phyCngRq = FALSE;
  connList[connIndex].isAutoPHYEnable = FALSE;

  return SUCCESS;
}

/*********************************************************************
 * @fn      SimplePeripheral_setPhy
 *
 * @brief   Call the HCI set phy API and and add the handle to a
 *          list to match it to an incoming command status event
 */
static status_t SimplePeripheral_setPhy(uint16_t connHandle, uint8_t allPhys,
                                        uint8_t txPhy, uint8_t rxPhy,
                                        uint16_t phyOpts)
{
  // Allocate list entry to store handle for command status
  spConnHandleEntry_t *connHandleEntry = ICall_malloc(sizeof(spConnHandleEntry_t));

  if (connHandleEntry)
  {
    connHandleEntry->connHandle = connHandle;

    // Add entry to the phy command status list
    List_put(&setPhyCommStatList, (List_Elem *)connHandleEntry);

    // Send PHY Update
    HCI_LE_SetPhyCmd(connHandle, allPhys, txPhy, rxPhy, phyOpts);
  }

  return SUCCESS;
}

/*********************************************************************
* @fn      SimplePeripheral_updatePHYStat
*
* @brief   Update the auto phy update state machine
*
* @param   connHandle - the connection handle
*
* @return  None
*/
static void SimplePeripheral_updatePHYStat(uint16_t eventCode, uint8_t *pMsg)
{
  uint8_t connIndex;

  switch (eventCode)
  {
    case HCI_LE_SET_PHY:
    {
      // Get connection handle from list
      spConnHandleEntry_t *connHandleEntry =
                           (spConnHandleEntry_t *)List_get(&setPhyCommStatList);

      if (connHandleEntry)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(connHandleEntry->connHandle);

        ICall_free(connHandleEntry);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          hciEvt_CommandStatus_t *pMyMsg = (hciEvt_CommandStatus_t *)pMsg;

          if (pMyMsg->cmdStatus == HCI_ERROR_CODE_UNSUPPORTED_REMOTE_FEATURE)
          {
            // Update the phychange request status for active RSSI tracking connection
            connList[connIndex].phyCngRq = FALSE;
            connList[connIndex].phyRqFailCnt++;
          }
        }
      }
      break;
    }

    // LE Event - a Phy update has completed or failed
    case HCI_BLE_PHY_UPDATE_COMPLETE_EVENT:
    {
      hciEvt_BLEPhyUpdateComplete_t *pPUC =
                                     (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

      if(pPUC)
      {
        // Get index from connection handle
        connIndex = SimplePeripheral_getConnIndex(pPUC->connHandle);

        // Is this connection still valid?
        if (connIndex < MAX_NUM_BLE_CONNS)
        {
          // Update the phychange request status for active RSSI tracking connection
          connList[connIndex].phyCngRq = FALSE;

          if (pPUC->status == SUCCESS)
          {
            connList[connIndex].currPhy = pPUC->rxPhy;
          }
          if(pPUC->rxPhy != connList[connIndex].rqPhy)
          {
            connList[connIndex].phyRqFailCnt++;
          }
          else
          {
            // Reset the request phy counter and requested phy
            connList[connIndex].phyRqFailCnt = 0;
            connList[connIndex].rqPhy = 0;
          }
        }
      }

      break;
    }

    default:
      break;
  } // end of switch (eventCode)
}

/*********************************************************************
*********************************************************************/
