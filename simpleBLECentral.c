/*******************************************************************************
  Filename:       SimpleBLECentral_Central.c
  Revised:        $Date: 2015-02-04 11:49:57 -0800 (Wed, 04 Feb 2015) $
  Revision:       $Revision: 42307 $

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the CC2650 Bluetooth Low Energy Protocol Stack.

  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <xdc/std.h>

#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

#include <ICall.h>
#include "bcomdef.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
//#include "uart.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_key.h"
#include "board_lcd.h"
#include "Board.h"
#include <driverlib/trng.h>

#include "simpleBLECentral.h"

#include <ti/drivers/lcd/LCDDogm1286.h>
/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * CONSTANTS
 */

// Simple BLE Central Task Events
#define SBC_START_DISCOVERY_EVT               0x0001
#define SBC_PAIRING_STATE_EVT                 0x0002
#define SBC_PASSCODE_NEEDED_EVT               0x0004
#define SBC_RSSI_READ_EVT                     0x0008
#define SBC_KEY_CHANGE_EVT                    0x0010
#define SBC_STATE_CHANGE_EVT                  0x0020

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 4000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is 
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      400

// Maximum connection interval (units of 1.25ms) if automatic parameter update
// request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update 
// request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           600

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

// Task configuration
#define SBC_TASK_PRIORITY                     1

#ifndef SBC_TASK_STACK_SIZE
#define SBC_TASK_STACK_SIZE                   864
#endif

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_MTU,                 // Exchange ATT MTU size
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct 
{
  uint8_t event;  // event type
  uint8_t status; // event status
  uint8_t *pData; // event data pointer
} sbcEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
/*********************************************************************************************************Yaswanth Malisetty code Begins*/
#define     Board_UART0             Board_UART
UART_Handle uart;
/*********************************************************************************************************Yaswanth Malisetty code Ends*/


// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock object used to signal timeout
static Clock_Struct startDiscClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

// Task pending events
static uint16_t events = 0;

// Task configuration
Task_Struct sbcTask;
Char sbcTaskStack[SBC_TASK_STACK_SIZE];

// GAP GATT Attributes
static const uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8_t scanRes;
static uint8_t scanIdx;

// Scan result list
static gapDevRec_t devList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static bool scanningStarted = FALSE;
//static bool addingStarted = FALSE;

// RSSI polling state
static bool rssiStarted = FALSE;

// Connection handle of current connection 
static uint16_t connHandle[10] = {GAP_CONNHANDLE_INIT};

// Application state
static uint8_t state = BLE_STATE_IDLE;

// Discovery state
static uint8_t discState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16_t svcStartHdl = 0;
static uint16_t svcEndHdl = 0;

// Discovered characteristic handle
static uint16_t charHdl[10] = {0};

// Value to write
static uint8_t charVal = 0;

// Value read/write toggle
static bool doWrite = FALSE;
bool listenAtUartRx=FALSE;

// GATT read/write procedure state
static bool procedureInProgress = FALSE;

// Callback variables
static uint8_t pairState;
static uint8_t pairStatus;
static uint8_t displayPasscode;

// Maximim PDU size (default = 27 octets)
static uint16 maxPduSize;

#ifdef TI_DRIVERS_LCD_INCLUDED
  static int8_t rssiValue;
#endif

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void SimpleBLECentral_init(void);
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1);

static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys);
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg);
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg);
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg);
static void SimpleBLECentral_startDiscovery(void);
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData, 
                                         uint8_t dataLen);
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType);
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status);
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs);

static void SimpleBLECentral_rssiCB(uint16_t connHandle, int8_t rssi);
static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent);
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs);
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state, 
                                         uint8_t status);

static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status, 
                                           uint8_t *pData);
char* itoa(uint8 val,char *buf, int base);
void SimpleBLECentral_startDiscHandler(UArg a0);
void SimpleBLECentral_keyChangeHandler(uint8 keysPressed);


/*********************************************************************************************************Yaswanth Malisetty code Begins*/

void ApplicationLayer_UartInit(void);
void ApplicationLayer_UartInit(void){
	UART_Params uartParams;
    const char echoPrompt[] = "\fUart Initialized\r\n";

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 9600;
    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        System_abort("Error opening the UART");
    }
  //  UART_write(uart, echoPrompt, sizeof(echoPrompt));
}
/*********************************************************************************************************Yaswanth Malisetty code Ends*/


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks


static gapCentralRoleCB_t SimpleBLECentral_roleCB =
{
  SimpleBLECentral_rssiCB,     // RSSI callback
  SimpleBLECentral_eventCB     // Event callback
};

// Bond Manager Callbacks
static gapBondCBs_t SimpleBLECentral_bondCB =
{
  SimpleBLECentral_passcodeCB, // Passcode callback
  SimpleBLECentral_pairStateCB // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   none
 *
 * @return  none
 */
void SimpleBLECentral_createTask(void)
{
  Task_Params taskParams;
    
  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbcTaskStack;
  taskParams.stackSize = SBC_TASK_STACK_SIZE;
  taskParams.priority = SBC_TASK_PRIORITY;
  
  Task_construct(&sbcTask, SimpleBLECentral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLECentral_init(void)
{
	// ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
	  ApplicationLayer_UartInit();

  ICall_registerApp(&selfEntity, &sem);
  // Hard code the DB Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 };
  //HCI_EXT_SetBDADDRCmd(bdAddress);
  
  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(40);
  
  // Create an RTOS queue for message from profile to be sent to app.
  appMsgQueue = Util_constructQueue(&appMsg);
     
  // Setup discovery delay as a one-shot timer
  Util_constructClock(&startDiscClock, SimpleBLECentral_startDiscHandler,
                      DEFAULT_SVC_DISCOVERY_DELAY, 0, false, 0);
  
  Board_initKeys(SimpleBLECentral_keyChangeHandler);

  Board_openLCD();
  
  // Setup Central Profile
  {
    uint8_t scanRes = DEFAULT_MAX_SCAN_RES;
    
    GAPCentralRole_SetParameter(GAPCENTRALROLE_MAX_SCAN_RES, sizeof(uint8_t), 
                                &scanRes);
  }
  
  // Setup GAP
  GAP_SetParamValue(TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GAP_SetParamValue(TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION);
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, 
                   (void *)attDeviceName);

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = DEFAULT_PASSCODE;
    uint8_t pairMode = DEFAULT_PAIRING_MODE;
//    uint8_t mitm = DEFAULT_MITM_MODE;
    uint8_t mitm = FALSE;
    uint8_t ioCap = DEFAULT_IO_CAPABILITIES;
    uint8_t bonding = DEFAULT_BONDING_MODE;
    
    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t), 
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd(selfEntity);

  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);         // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
  
  // Start the Device
  VOID GAPCentralRole_StartDevice(&SimpleBLECentral_roleCB);

  // Register with bond manager after starting device
  GAPBondMgr_Register(&SimpleBLECentral_bondCB);
  
  LCD_WRITE_STRING("BLE Central", LCD_PAGE0);
}
char* itoa(uint8 val,char *buf, int base)
{
int i = 30;
for(; val && i ; --i, val /= base)
buf[i] = "0123456789abcdef"[val % base];
return &buf[i+1];
}




/*********************************************************************
 * @fn      SimpleBLECentral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Central.
 *
 * @param   none
 *
 * @return  events not processed
 */
static void SimpleBLECentral_taskFxn(UArg a0, UArg a1)
{
  // Initialize application
	char input;
    const char echoPrompt[] = "\nUart recieved\r\n";
  SimpleBLECentral_init();
  
  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {

      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;
      
      if (ICall_fetchServiceMsg(&src, &dest, 
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          // Process inter-task message
          SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg);
        }

        if (pMsg)
        {
          ICall_freeMsg(pMsg);
        }
      }
    }

    // If RTOS queue is not empty, process app message
    if (!Queue_empty(appMsgQueue))
    {
      sbcEvt_t *pMsg = (sbcEvt_t *)Util_dequeueMsg(appMsgQueue);
      if (pMsg)
      {
        // Process message
        SimpleBLECentral_processAppMsg(pMsg);
        
        // Free the space from the message
        ICall_free(pMsg);
      }
    }
    
    if (events & SBC_START_DISCOVERY_EVT)
    {      
      events &= ~SBC_START_DISCOVERY_EVT;
      
      SimpleBLECentral_startDiscovery();
    }
    
    if (events & SBC_PAIRING_STATE_EVT)
    {      
      events &= ~SBC_PAIRING_STATE_EVT;
      
      SimpleBLECentral_processPairState(pairState, pairStatus);
    }
         
    if (events & SBC_RSSI_READ_EVT)
    {      
      events &= ~SBC_RSSI_READ_EVT;

      LCD_WRITE_STRING_VALUE("RSSI -dB:", (uint32_t)(-rssiValue), 10, LCD_PAGE4);
    }
    
    if (events & SBC_PASSCODE_NEEDED_EVT)
    {      
      events &= ~SBC_PASSCODE_NEEDED_EVT;
      
      SimpleBLECentral_processPasscode(connHandle[scanIdx], displayPasscode);
    }
    if((UARTCharsAvail(Board_UART0)&&procedureInProgress == FALSE)&&listenAtUartRx)
     {
       uint8_t status;
       UART_read(uart, &input, 1);
       UART_write(uart,&input,1);
      /* if(input=='R'){
    	   attReadReq_t req;
    	   req.handle = charHdl[scanIdx];
    	   status = GATT_ReadCharValue(connHandle[scanIdx], &req, selfEntity);
       }
       */if(input=='A'){ // Read BLE 1 Led status
    	   attReadReq_t req;
    	   req.handle = charHdl[0];
    	   status = GATT_ReadCharValue(connHandle[scanIdx], &req, selfEntity);
       }else if(input=='B'){ // Read BLE 2 Led status
    	   attReadReq_t req;
    	   req.handle = charHdl[1];
    	   status = GATT_ReadCharValue(connHandle[scanIdx], &req, selfEntity);
       }else{
    	   uint8 numAllocConns=0,numActiveConns=0;
    		  HCI_EXT_GetNumConnsCmd((uint8 *)&numAllocConns,(uint8 *)&numActiveConns);
    		  char def[32]={0},connec[32]={0};
    		  char *str=itoa(numAllocConns,def, 10);
    		  char *act=itoa(numActiveConns,connec, 10);
    		  itoa(numActiveConns,connec, 10);
    		  //printf(def,"%d",&numAllocConns);
    		  //printf(connec,"%d",&numActiveConns);
    		  UART_write(uart,str,strlen(str));
    		  UART_write(uart," ",strlen("\n"));
    		  UART_write(uart,act,strlen(act));
    		  UART_write(uart," ",strlen("\n"));
    		  uint8_t wrIndex=0;
     	    //UART_write(uart, echoPrompt, sizeof(echoPrompt));
  	            // Do a read or write as long as no other read or write is in progress
     	          attWriteReq_t req;

     	         // UART_write(uart, "sending write req\n", 18);
     	          if(input>='1'&&input<'3'){
     	        		input=input;
     	        		 UART_write(uart,"wrIndex=0",strlen("wrIndex=0"));
    		    	    wrIndex=0;
     	          }else if(input>'2'&&input<'5'){
      	        		input=input-2;
    	        		 UART_write(uart,"wrIndex=1",strlen("wrIndex=0"));
      	        	    wrIndex=1;
         	      }else if(input>'4'&&input<'7'){
  	        		 UART_write(uart,"wrIndex=2",strlen("wrIndex=0"));
       	        		input=input-4;
       	        	    wrIndex=2;
     	          }
     	          req.pValue = GATT_bm_alloc(connHandle[wrIndex], ATT_WRITE_REQ, 1, NULL);
     	          if ( req.pValue != NULL )
     	          {
     	        	req.handle = charHdl[wrIndex];
     	            req.len = 1;
     	            req.pValue[0] = input;
     	           charVal=input;
     	            req.sig = 0;
     	            req.cmd = 0;

     	            status = GATT_WriteCharValue(connHandle[wrIndex], &req, selfEntity);
     	            if ( status != SUCCESS )
     	            {
     	              GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
     	            }
     	          }
       	        if (status == SUCCESS)
     	        {
     	          procedureInProgress = TRUE;
     	          doWrite = !doWrite;
     	        }
     	 }
     	//PIN_setOutputValue(ledPinHandle, Board_LED1, 0);
       //UART_write(uart,&input,1);
     }//if uart available
   }//for
}

/*********************************************************************
 * @fn      SimpleBLECentral_processStackMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void SimpleBLECentral_processStackMsg(ICall_Hdr *pMsg)
{
  switch (pMsg->event)
  {
    case GAP_MSG_EVENT:
      SimpleBLECentral_processRoleEvent((gapCentralRoleEvent_t *)pMsg);
      break;
      
    case GATT_MSG_EVENT:
      SimpleBLECentral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processAppMsg
 *
 * @brief   Central application event processing function.
 *
 * @param   pMsg - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processAppMsg(sbcEvt_t *pMsg)
{
  switch (pMsg->event)
  {
    case SBC_STATE_CHANGE_EVT:
      SimpleBLECentral_processStackMsg((ICall_Hdr *)pMsg->pData);
      
      // Free the stack message
      ICall_freeMsg(pMsg->pData);
      break;
      
    case SBC_KEY_CHANGE_EVT:
      SimpleBLECentral_handleKeys(0, pMsg->status); 
      break;
      
    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processRoleEvent
 *
 * @brief   Central role event processing function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void SimpleBLECentral_processRoleEvent(gapCentralRoleEvent_t *pEvent)
{
	UART_write(uart,"\r\n\f SimpleBLECentral_processRoleEvent:",strlen("\r\n\f SimpleBLECentral_processRoleEvent:"));
  switch (pEvent->gap.opcode)
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        maxPduSize = pEvent->initDone.dataPktLen;
        
        LCD_WRITE_STRING(Util_convertBdAddr2Str(pEvent->initDone.devAddr),
                         LCD_PAGE1);
        LCD_WRITE_STRING("Initialized", LCD_PAGE2);

        //*********************************************************************************added code starts here by Yash
        if (state != BLE_STATE_CONNECTED)
          {
             	        if (!scanningStarted)
             	        {
             	          scanningStarted = TRUE;
             	          scanRes = 0;

             	          LCD_WRITE_STRING("Discovering...", LCD_PAGE2);
             	          LCD_WRITE_STRING("", LCD_PAGE3);
             	          LCD_WRITE_STRING("", LCD_PAGE4);

             	          GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
             	                                        DEFAULT_DISCOVERY_ACTIVE_SCAN,
             	                                        DEFAULT_DISCOVERY_WHITE_LIST);
             	        }
          }
        //*********************************************************************************added code ends here by Yash


      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
    	  UART_write(uart, "\nGAP_DEVICE_INFO_EVENT\n", 23);
        // if filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE)
        {
          if (SimpleBLECentral_findSvcUuid(SIMPLEPROFILE_SERV_UUID,
                                           pEvent->deviceInfo.pEvtData,
                                           pEvent->deviceInfo.dataLen))
          {
            SimpleBLECentral_addDeviceInfo(pEvent->deviceInfo.addr, 
                                           pEvent->deviceInfo.addrType);
          }
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
    	  UART_write(uart, "\fGAP_DEVICE_DISCOVERY_EVENT\n", 28);
    	  // discovery complete
        scanningStarted = FALSE;

        // if not filtering device discovery results based on service UUID
        if (DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE)
        {
          // Copy results
          scanRes = pEvent->discCmpl.numDevs;
          memcpy(devList, pEvent->discCmpl.pDevList,
                 (sizeof(gapDevRec_t) * scanRes));
        }
        
        LCD_WRITE_STRING_VALUE("Devices Found", scanRes, 10, LCD_PAGE2);
        
        if (scanRes > 0)
        {
          LCD_WRITE_STRING("<- To Select", LCD_PAGE3);
          char def[32]={0};
          char *str=itoa(scanRes,def, 10);
          UART_write(uart,"found:",strlen("found:"));
          UART_write(uart,str,strlen(str));
      //    int i=0;
          /*for(i=0;i<scanRes;i++){
        	 UART_write(uart,Util_convertBdAddr2Str(devList[i].addr),strlen(Util_convertBdAddr2Str(devList[i].addr)));
        	 uint8_t addrType;
        	 uint8_t *peerAddr;
        	 peerAddr = devList[i].addr;
			 addrType = devList[i].addrType;

			 state = BLE_STATE_CONNECTING;
			// while(addingStarted);
			 GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
										  DEFAULT_LINK_WHITE_LIST,
										  addrType, peerAddr);
			 addingStarted=TRUE;
			 UART_write(uart,"\fAfterAdding",strlen("\fAfterAdding"));
          }
	*/
          //*********************************************************************************added code starts here by Yash
          if (!scanningStarted && scanRes > 0)
           {
             // Increment index of current result (with wraparound)
             scanIdx=0;
             if (scanIdx >= scanRes)
             {
               scanIdx = 0;
             }

             LCD_WRITE_STRING_VALUE("Device", (scanIdx + 1), 10, LCD_PAGE2);

             LCD_WRITE_STRING(Util_convertBdAddr2Str(devList[scanIdx].addr), LCD_PAGE3);

             if (state == BLE_STATE_IDLE)
                        {
                          // if there is a scan result
                          if (scanRes > 0)
                          {
                            // connect to current device in scan result
                        	  uint8_t addrType;
                        	  uint8_t *peerAddr;

                        	  peerAddr = devList[scanIdx].addr;
                            addrType = devList[scanIdx].addrType;

                            state = BLE_STATE_CONNECTING;

                            GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                                         DEFAULT_LINK_WHITE_LIST,
                                                         addrType, peerAddr);

                            LCD_WRITE_STRING("Connecting", LCD_PAGE2);
                            LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddr), LCD_PAGE3);
                            LCD_WRITE_STRING("", LCD_PAGE4);
                          }
             }
           }
          //*********************************************************************************added code ends here by Yash

        }

        // initialize scan index to last device
        //scanIdx = scanRes;
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
    	  UART_write(uart, "\fGAP_LINK_ESTABLISHED_EVENT\n", 28);

        if (pEvent->gap.hdr.status == SUCCESS)
        {
          state = BLE_STATE_CONNECTED;
          connHandle[scanIdx] = pEvent->linkCmpl.connectionHandle;
          procedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          if (charHdl[scanIdx] == 0)
          {
            Util_startClock(&startDiscClock);
          }

          LCD_WRITE_STRING("Connected", LCD_PAGE2);
          LCD_WRITE_STRING(Util_convertBdAddr2Str(pEvent->linkCmpl.devAddr),
                           LCD_PAGE3);   
        }
        else
        {
          state = BLE_STATE_IDLE;
          connHandle[scanIdx] = GAP_CONNHANDLE_INIT;
          rssiStarted = FALSE;
          discState = BLE_DISC_STATE_IDLE;
          
          LCD_WRITE_STRING("Connect Failed", LCD_PAGE2);
          LCD_WRITE_STRING_VALUE("Reason:", pEvent->gap.hdr.status, 10, 
                                 LCD_PAGE3);
        }



      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        state = BLE_STATE_IDLE;
        connHandle[scanIdx] = GAP_CONNHANDLE_INIT;
        rssiStarted = FALSE;
        discState = BLE_DISC_STATE_IDLE;
        charHdl[scanIdx] = 0;
        procedureInProgress = FALSE;
          
        LCD_WRITE_STRING("Disconnected", LCD_PAGE2);
        LCD_WRITE_STRING_VALUE("Reason:", pEvent->linkTerminate.reason,
                                10, LCD_PAGE3);
        LCD_WRITE_STRING("", LCD_PAGE4);
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
    	  UART_write(uart, "\fGAP_LINK_PARAM_UPDATE_EVENT\n", 29);
    	  listenAtUartRx=TRUE;
    	  LCD_WRITE_STRING_VALUE("Param Update:", pEvent->linkUpdate.status,
                                10, LCD_PAGE2);
          char def[32]={0};
          char *str=itoa(scanRes,def, 10);
          UART_write(uart,"scanRes:",strlen("scanRes:"));
          UART_write(uart,str,strlen(str));
          {
        	  char def[32]={0};
        	  char *str=itoa(scanIdx,def, 10);
			UART_write(uart,"scanIdx:",strlen("scanIdx:"));
			UART_write(uart,str,strlen(str));

          }
          //*********************************************************************************added code starts here by Yash
    	        if (scanIdx < scanRes-1)
    	                    {

    	        			scanIdx++;

    	                    LCD_WRITE_STRING_VALUE("Device", (scanIdx + 1), 10, LCD_PAGE2);

    	                    LCD_WRITE_STRING(Util_convertBdAddr2Str(devList[scanIdx].addr), LCD_PAGE3);

    	                                 // if there is a scan result
    	                                 if (scanRes > 0)
    	                                 {
    	                                	  uint8_t addrType;
    	                                                       	  uint8_t *peerAddr;
    	                                 // connect to current device in scan result
    	                                   peerAddr = devList[scanIdx].addr;
    	                                   addrType = devList[scanIdx].addrType;

    	                                   state = BLE_STATE_CONNECTING;
    	           	        			UART_write(uart,"connecting..",strlen("connecting.."));
    	                                   GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
    	                                                                DEFAULT_LINK_WHITE_LIST,
    	                                                                addrType, peerAddr);

    	                                   UART_write(uart,"connecting.2",strlen("connecting.."));

    	                                   // added for fixing blocking isuue
    	                                   attWriteReq_t req;
										   req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 1, NULL);
										   if ( req.pValue != NULL )
										   {
											 req.handle = charHdl[0];
											 req.len = 1;
											 req.pValue[0] = charVal;
											 req.sig = 0;
											 req.cmd = 0;
											 uint8_t status;
											status = GATT_WriteCharValue(connHandle, &req, selfEntity);
											if ( status != SUCCESS )
											{
											   GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
											}
    	                                   }
										   //end of bug blocking issue

    	                                   LCD_WRITE_STRING("Connecting", LCD_PAGE2);
    	                                   LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddr), LCD_PAGE3);
    	                                   LCD_WRITE_STRING("", LCD_PAGE4);
    	                                 }

    	                 }
    	       //*********************************************************************************added code ends here by Ya
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void SimpleBLECentral_handleKeys(uint8_t shift, uint8_t keys)
{
  (void)shift;  // Intentionally unreferenced parameter

  if (keys & KEY_LEFT)
  {
    // Display discovery results
    if (!scanningStarted && scanRes > 0)
    {
      // Increment index of current result (with wraparound)
      scanIdx++;
      if (scanIdx >= scanRes)
      {
        scanIdx = 0;
      }

      LCD_WRITE_STRING_VALUE("Device", (scanIdx + 1), 10, LCD_PAGE2);
      LCD_WRITE_STRING(Util_convertBdAddr2Str(devList[scanIdx].addr), LCD_PAGE3);
    }

    return;
  }

  if (keys & KEY_UP)
  {
    // Start or stop discovery
    if (state != BLE_STATE_CONNECTED)
    {
      if (!scanningStarted)
      {
        scanningStarted = TRUE;
        scanRes = 0;
        
        LCD_WRITE_STRING("Discovering...", LCD_PAGE2);
        LCD_WRITE_STRING("", LCD_PAGE3);
        LCD_WRITE_STRING("", LCD_PAGE4);
        
        GAPCentralRole_StartDiscovery(DEFAULT_DISCOVERY_MODE,
                                      DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                      DEFAULT_DISCOVERY_WHITE_LIST);      
      }
      else
      {
        GAPCentralRole_CancelDiscovery();
      }
    }
    else if (state == BLE_STATE_CONNECTED &&
             charHdl[scanIdx] != 0                 &&
             procedureInProgress == FALSE)
    {
      uint8_t status;

      // Do a read or write as long as no other read or write is in progress
      if (doWrite)
      {
        // Do a write
        attWriteReq_t req;

        req.pValue = GATT_bm_alloc(connHandle, ATT_WRITE_REQ, 1, NULL);
        if ( req.pValue != NULL )
        {
          req.handle = charHdl[scanIdx];
          req.len = 1;
          req.pValue[0] = charVal;
          req.sig = 0;
          req.cmd = 0;

          status = GATT_WriteCharValue(connHandle, &req, selfEntity);
          if ( status != SUCCESS )
          {
            GATT_bm_free((gattMsg_t *)&req, ATT_WRITE_REQ);
          }
        }
      }
      else
      {
        // Do a read
        attReadReq_t req;
        
        req.handle = charHdl[scanIdx];
        status = GATT_ReadCharValue(connHandle, &req, selfEntity);
      }

      if (status == SUCCESS)
      {
        procedureInProgress = TRUE;
        doWrite = !doWrite;
      }
    }

    return;
  }

  if (keys & KEY_RIGHT)
  {
    // Connection update
    if (state == BLE_STATE_CONNECTED)
    {
      GAPCentralRole_UpdateLink(connHandle,
                                DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                DEFAULT_UPDATE_SLAVE_LATENCY,
                                DEFAULT_UPDATE_CONN_TIMEOUT);
    }
    
    return;
  }

  if (keys & KEY_SELECT)
  {
    uint8_t addrType;
    uint8_t *peerAddr;
    
    // Connect or disconnect
    if (state == BLE_STATE_IDLE)
    {
      // if there is a scan result
      if (scanRes > 0)
      {
        // connect to current device in scan result
        peerAddr = devList[scanIdx].addr;
        addrType = devList[scanIdx].addrType;
      
        state = BLE_STATE_CONNECTING;
        
        GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                     DEFAULT_LINK_WHITE_LIST,
                                     addrType, peerAddr);
  
        LCD_WRITE_STRING("Connecting", LCD_PAGE2);
        LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddr), LCD_PAGE3);
        LCD_WRITE_STRING("", LCD_PAGE4);
      }
    }
    else if (state == BLE_STATE_CONNECTING ||
              state == BLE_STATE_CONNECTED)
    {
      // disconnect
      state = BLE_STATE_DISCONNECTING;

      GAPCentralRole_TerminateLink(connHandle);
      
      LCD_WRITE_STRING("Disconnecting", LCD_PAGE2);
      LCD_WRITE_STRING("", LCD_PAGE4);
    }

    return;
  }

  if (keys & KEY_DOWN)
  {
    // Start or cancel RSSI polling
    if (state == BLE_STATE_CONNECTED)
    {
      if (!rssiStarted)
      {
        rssiStarted = TRUE;
        GAPCentralRole_StartRssi(connHandle, DEFAULT_RSSI_PERIOD);
      }
      else
      {
        rssiStarted = FALSE;
        GAPCentralRole_CancelRssi(connHandle);

        LCD_WRITE_STRING("RSSI Cancelled", LCD_PAGE4);
      }
    }

    return;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  if (state == BLE_STATE_CONNECTED)
  {
    if ((pMsg->method == ATT_READ_RSP)   ||
        ((pMsg->method == ATT_ERROR_RSP) &&
         (pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ)))
    {
      if (pMsg->method == ATT_ERROR_RSP)
      {      
        LCD_WRITE_STRING_VALUE("Read Error", pMsg->msg.errorRsp.errCode, 10,
                               LCD_PAGE4);
      }
      else
      {
        // After a successful read, display the read value
        LCD_WRITE_STRING_VALUE("Read rsp:", pMsg->msg.readRsp.pValue[0], 10,
                               LCD_PAGE4);
        uint8_t val=pMsg->msg.readRsp.pValue[0];
        UART_write(uart,&val,1);
      }
      
      procedureInProgress = FALSE;
    }
    else if ((pMsg->method == ATT_WRITE_RSP)  ||
             ((pMsg->method == ATT_ERROR_RSP) &&
              (pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ)))
    {
      
      if (pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP)
      {     
        LCD_WRITE_STRING_VALUE("Write Error", pMsg->msg.errorRsp.errCode, 10,
                               LCD_PAGE4);
      }
      else
      {
        // After a succesful write, display the value that was written and
        // increment value
        //LCD_WRITE_STRING_VALUE("Write sent:", charVal++, 10, LCD_PAGE4);
    	  LCD_WRITE_STRING_VALUE("write rsp:",charVal, 10,
    	                                 LCD_PAGE4);
    	  //UART_write(uart,&charVal,1);
      }
      
      procedureInProgress = FALSE;    

    }
    else if (discState != BLE_DISC_STATE_IDLE)
    {
      SimpleBLECentral_processGATTDiscEvent(pMsg);
    }
  } // else - in case a GATT message came after a connection has dropped, ignore it.
  
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      SimpleBLECentral_eventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  TRUE if safe to deallocate event message, FALSE otherwise.
 */
static uint8_t SimpleBLECentral_eventCB(gapCentralRoleEvent_t *pEvent)
{
  // Forward the role event to the application
  if (SimpleBLECentral_enqueueMsg(SBC_STATE_CHANGE_EVT, 
                                  SUCCESS, (uint8_t *)pEvent))
  {
    // App will process and free the event
    return FALSE;
  }
  
  // Caller should free the event
  return TRUE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_rssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void SimpleBLECentral_rssiCB(uint16_t connHandle, int8_t rssi)
{
#ifdef TI_DRIVERS_LCD_INCLUDED
  // Remember the latest RSSI value for display
  rssiValue = rssi;
#endif
  
  events |= SBC_RSSI_READ_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLECentral_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void SimpleBLECentral_pairStateCB(uint16_t connHandle, uint8_t state,
                                         uint8_t status)
{
  // Remember the new state and status
	UART_write(uart,"\r\n\f SimpleBLECentral_pairStateCB:",strlen("\r\n\f SimpleBLECentral_pairStateCB:"));

  pairState = state;
  pairStatus = status;
  
  events |= SBC_PAIRING_STATE_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLECentral_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void SimpleBLECentral_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                                        uint8_t uiInputs, uint8_t uiOutputs)
{
  displayPasscode = uiOutputs;
  
  events |= SBC_PASSCODE_NEEDED_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPairState
 *
 * @brief   Process the new paring state.
 *
 * @return  none
 */
static void SimpleBLECentral_processPairState(uint8_t state, uint8_t status)
{
  if (state == GAPBOND_PAIRING_STATE_STARTED)
  {

    LCD_WRITE_STRING("Pairing started", LCD_PAGE2);
  }
  else if (state == GAPBOND_PAIRING_STATE_COMPLETE)
  {
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING("Pairing success", LCD_PAGE2);
    }
    else
    {
      LCD_WRITE_STRING_VALUE("Pairing fail:", status, 10, LCD_PAGE2);
    }
  }
  else if (state == GAPBOND_PAIRING_STATE_BONDED)
  {
    if (status == SUCCESS)
    {
      LCD_WRITE_STRING("Bonding success", LCD_PAGE2);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_processPasscode
 *
 * @brief   Process the Passcode request.
 *
 * @return  none
 */
static void SimpleBLECentral_processPasscode(uint16_t connectionHandle,
                                             uint8_t uiOutputs)
{
  uint32_t  passcode;

  // Create random passcode
  passcode = Util_GetTRNG();
  passcode %= 1000000;

  // Display passcode to user
  if (uiOutputs != 0)
  {
    LCD_WRITE_STRING_VALUE("Passcode:", passcode, 10, LCD_PAGE4);
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp(connectionHandle, SUCCESS, passcode);
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void SimpleBLECentral_startDiscovery(void)
{
  attExchangeMTUReq_t req;
  
  // Initialize cached handles
  svcStartHdl = svcEndHdl = charHdl[scanIdx] = 0;
    
  discState = BLE_DISC_STATE_MTU;
  
  // Discover GATT Server's Rx MTU size
  req.clientRxMTU = maxPduSize - L2CAP_HDR_SIZE;
  
  // ATT MTU size should be set to the minimum of the Client Rx MTU
  // and Server Rx MTU values
  VOID GATT_ExchangeMTU(connHandle[scanIdx], &req, selfEntity);
}

/*********************************************************************
 * @fn      SimpleBLECentral_processGATTDiscEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void SimpleBLECentral_processGATTDiscEvent(gattMsgEvent_t *pMsg)
{ 
	UART_write(uart,"\r\n\f SimpleBLECentral_processGATTDiscEvent:",strlen("\r\n\f SimpleBLECentral_processGATTDiscEvent:"));
  if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {   
    // MTU size updated
    LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE4);
  }
  else if (discState == BLE_DISC_STATE_MTU)
  {
    // MTU size response received, discover simple BLE service
    if (pMsg->method == ATT_EXCHANGE_MTU_RSP)
    {
      uint8_t uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                         HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
      
      // Just in case we're using the default MTU size (23 octets)
      LCD_WRITE_STRING_VALUE("MTU Size:", ATT_MTU_SIZE, 10, LCD_PAGE4);
        
      discState = BLE_DISC_STATE_SVC;

      // Discovery simple BLE service
      VOID GATT_DiscPrimaryServiceByUUID(connHandle[scanIdx], uuid, ATT_BT_UUID_SIZE,
                                         selfEntity);
    }
  }
  else if (discState == BLE_DISC_STATE_SVC)
  {
    // Service found, store handles
    if (pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
        pMsg->msg.findByTypeValueRsp.numInfo > 0)
    {
      svcStartHdl = ATT_ATTR_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
      svcEndHdl = ATT_GRP_END_HANDLE(pMsg->msg.findByTypeValueRsp.pHandlesInfo, 0);
    }
    
    // If procedure complete
    if (((pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP) && 
         (pMsg->hdr.status == bleProcedureComplete))  ||
        (pMsg->method == ATT_ERROR_RSP))
    {
      if (svcStartHdl != 0)
      {
        attReadByTypeReq_t req;
          
        // Discover characteristic
        discState = BLE_DISC_STATE_CHAR;
        
        req.startHandle = svcStartHdl;
        req.endHandle = svcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        VOID GATT_ReadUsingCharUUID(connHandle[scanIdx], &req, selfEntity);
      }
    }
  }
  else if (discState == BLE_DISC_STATE_CHAR)
  {
    // Characteristic found, store handle
    if ((pMsg->method == ATT_READ_BY_TYPE_RSP) && 
        (pMsg->msg.readByTypeRsp.numPairs > 0))
    {

    	charHdl[scanIdx] = BUILD_UINT16(pMsg->msg.readByTypeRsp.pDataList[0],
                             pMsg->msg.readByTypeRsp.pDataList[1]);
      UART_write(uart," writing charHdl:",strlen(" writing charHdl:"));
      if(!scanIdx){
    	  UART_write(uart," writing charHdl:0",strlen(" writing charHdl:0"));
      }else{
    	  UART_write(uart," writing charHdl:n",strlen(" writing charHdl:n"));
      }
      LCD_WRITE_STRING("Simple Svc Found", LCD_PAGE2);
      procedureInProgress = FALSE;
    }
    
    discState = BLE_DISC_STATE_IDLE;
  }    
}

/*********************************************************************
 * @fn      SimpleBLECentral_findSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool SimpleBLECentral_findSvcUuid(uint16_t uuid, uint8_t *pData, 
                                         uint8_t dataLen)
{
	UART_write(uart,"\r\n\f SimpleBLECentral_findSvcUuid:",strlen("\r\n\f SimpleBLECentral_findSvcUuid:"));

  uint8_t adLen;
  uint8_t adType;
  uint8_t *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while (pData < pEnd)
  {
    // Get length of next AD item
    adLen = *pData++;
    if (adLen > 0)
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ((adType == GAP_ADTYPE_16BIT_MORE) || 
          (adType == GAP_ADTYPE_16BIT_COMPLETE))
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while (adLen >= 2 && pData < pEnd)
        {
          // Check for match
          if ((pData[0] == LO_UINT16(uuid)) && (pData[1] == HI_UINT16(uuid)))
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if (adLen == 1)
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      SimpleBLECentral_addDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void SimpleBLECentral_addDeviceInfo(uint8_t *pAddr, uint8_t addrType)
{
  uint8_t i;
  
  // If result count not at max
  if (scanRes < DEFAULT_MAX_SCAN_RES)
  {
    // Check if device is already in scan results
    for (i = 0; i < scanRes; i++)
    {
      if (memcmp(pAddr, devList[i].addr , B_ADDR_LEN) == 0)
      {
        return;
      }
    }
    
    // Add addr to scan result list
    memcpy(devList[scanRes].addr, pAddr, B_ADDR_LEN);
    devList[scanRes].addrType = addrType;
    
    // Increment scan result count
    scanRes++;
  }
}

/*********************************************************************
 * @fn      SimpleBLECentral_startDiscHandler
 *
 * @brief   Clock handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_startDiscHandler(UArg a0)
{
  events |= SBC_START_DISCOVERY_EVT;

  // Wake up the application thread when it waits for clock event
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLECentral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
void SimpleBLECentral_keyChangeHandler(uint8 keys)
{
  SimpleBLECentral_enqueueMsg(SBC_KEY_CHANGE_EVT, keys, NULL);
}

/*********************************************************************
 * @fn      SimpleBLECentral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   status - message status.
 * @param   pData - message data pointer.
 *
 * @return  TRUE or FALSE
 */
static uint8_t SimpleBLECentral_enqueueMsg(uint8_t event, uint8_t status, 
                                           uint8_t *pData)
{
  sbcEvt_t *pMsg = ICall_malloc(sizeof(sbcEvt_t));
  
  // Create dynamic pointer to message.
  if (pMsg)
  {
    pMsg->event = event;
    pMsg->status = status;
    pMsg->pData = pData;
    
    // Enqueue the message.
    return Util_enqueueMsg(appMsgQueue, sem, (uint8_t *)pMsg);
  }
  
  return FALSE;
}

/*********************************************************************
*********************************************************************/
