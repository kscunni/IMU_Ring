/******************************************************************************

@file  app_peripheral.c

@brief This example file demonstrates how to activate the peripheral role with
the help of BLEAppUtil APIs.

Two structures are used for event handling, one for connection events and one
for advertising events.
In each, eventMask is used to specify the events that will be received
and handled.
In addition, fill the BLEAppUtil_AdvInit_t structure with variables generated
by the Sysconfig.

In the events handler functions, write what actions are done after each event.
In this example, after a connection is made, activation is performed for
re-advertising up to the maximum connections.

In the Peripheral_start() function at the bottom of the file, registration,
initialization and activation are done using the BLEAppUtil API functions,
using the structures defined in the file.

More details on the functions and structures can be seen next to the usage.

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2026, Texas Instruments Incorporated
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

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti_ble_config.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/app_util/menu/menu_module.h"
#include <app_main.h>
#include <ti/drivers/dpl/ClockP.h>

//*****************************************************************************
//! Prototypes
//*****************************************************************************
void Peripheral_AdvEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
void Peripheral_GAPConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************

BLEAppUtil_EventHandler_t peripheralConnHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_CONN_TYPE,
    .pEventHandler  = Peripheral_GAPConnEventHandler,
    .eventMask      = BLEAPPUTIL_LINK_ESTABLISHED_EVENT |
                      BLEAPPUTIL_LINK_PARAM_UPDATE_REQ_EVENT |
                      BLEAPPUTIL_LINK_TERMINATED_EVENT,
};

BLEAppUtil_EventHandler_t peripheralAdvHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_ADV_TYPE,
    .pEventHandler  = Peripheral_AdvEventHandler,
    .eventMask      = BLEAPPUTIL_ADV_START_AFTER_ENABLE |
                      BLEAPPUTIL_ADV_END_AFTER_DISABLE
};

//! Stores adv handles
uint8_t peripheralAdvHandles[BLE_CONFIG_NUM_ADV_SETS];

//! For param update delay
static ClockP_Struct paramUpdateClock;
//! handle for connection to send param update req to
static uint16_t ConnHandle = 0xFFFF;

//*****************************************************************************
//! Functions
//*****************************************************************************

static void requestConnParamUpdate(char *data)
{
    // gapUpdateLinkParamReq_t req =
    // {
    //     .connectionHandle = connHandle,

    //     // 24 * 1.25ms = 30ms
    //     .intervalMin = 24,
    //     .intervalMax = 24,

    //     .connLatency = 0,

    //     // 500 * 10ms = 5s supervision timeout
    //     .connTimeout = 500
    // };

    // GAP_UpdateLinkParamReq(&req);

    bStatus_t status;
    gapUpdateLinkParamReq_t pParamUpdateReq =
    {
     .connectionHandle = ConnHandle,
     .intervalMin = DEFAULT_DESIRED_MIN_CONN_INTERVAL,
     .intervalMax = DEFAULT_DESIRED_MAX_CONN_INTERVAL,
     .connLatency = DEFAULT_DESIRED_PERIPHERAL_LATENCY,
     .connTimeout = DEFAULT_DESIRED_CONN_TIMEOUT
    };

    // Send a connection param update request
    status = GAP_UpdateLinkParamReq(&pParamUpdateReq);
}

// 2a. This triggers when the clock expires (Swi Context!)
static void App_paramUpdateClockHandler(uintptr_t arg)
{
    // Offload the actual GAP call to the main application task thread
    BLEAppUtil_invokeFunction(requestConnParamUpdate, NULL);
}

void App_initParamUpdateClock(void)
{
    ClockP_Params clockParams;
    ClockP_Params_init(&clockParams);
    
    // Set to false so it doesn't start automatically
    clockParams.startFlag = false; 
    
    // For a one-shot timer, period must be 0 
    // (ClockP_Params_init sets this by default, but it is good to be explicit)
    clockParams.period = 0; 
    
    // Convert 2000 milliseconds to system ticks
    uint32_t timeoutTicks = (2000 * 1000) / ClockP_getSystemTickPeriod(); 
    
    // Construct the clock object - Notice timeoutTicks is the 3rd argument!
    ClockP_construct(&paramUpdateClock, 
                     (ClockP_Fxn)App_paramUpdateClockHandler, 
                     timeoutTicks,  
                     &clockParams);
}





/*********************************************************************
 * @fn      Peripheral_AdvEventHandler
 *
 * @brief   The purpose of this function is to handle advertise events
 *          that rise from the GAP and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Peripheral_AdvEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_ADV_START_AFTER_ENABLE:
        {
            MenuModule_printf(APP_MENU_ADV_EVENT, 0, "Adv status: Started - handle: "
                              MENU_MODULE_COLOR_YELLOW "%d" MENU_MODULE_COLOR_RESET,
                              ((BLEAppUtil_AdvEventData_t *)pMsgData)->pBuf->advHandle);
            break;
        }

        case BLEAPPUTIL_ADV_END_AFTER_DISABLE:
        {
            MenuModule_printf(APP_MENU_ADV_EVENT, 0, "Adv status: Ended - handle: "
                              MENU_MODULE_COLOR_YELLOW "%d" MENU_MODULE_COLOR_RESET,
                              ((BLEAppUtil_AdvEventData_t *)pMsgData)->pBuf->advHandle);
            break;
        }

        default:
        {
            break;
        }
    }
}

/*********************************************************************
 * @fn      Peripheral_GAPConnEventHandler
 *
 * @brief   The purpose of this function is to handle connection related
 *          events that rise from the GAP and were registered in
 *          @ref BLEAppUtil_registerEventHandler
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void Peripheral_GAPConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
        {
            //! UNCOMMENT HERE
            // gapEstLinkReqEvent_t *pPkt = (gapEstLinkReqEvent_t *)pMsgData;
            // ConnHandle = (pPkt->connectionHandle);
            // ClockP_start(ClockP_handle(&paramUpdateClock));
            /* Check if we reach the maximum allowed number of connections */
            if(linkDB_NumActive() < linkDB_NumConns())
            {
                /* Start advertising since there is room for more connections */
                BleConfig_startAdvSets(peripheralAdvHandles, NULL, BLE_CONFIG_NUM_ADV_SETS);
            }
            else
            {
                /* Stop advertising since there is no room for more connections */
                BleConfig_stopAdvSets(peripheralAdvHandles, NULL, BLE_CONFIG_NUM_ADV_SETS);
            }
            break;
        }

        case BLEAPPUTIL_LINK_TERMINATED_EVENT:
        {
            BleConfig_startAdvSets(peripheralAdvHandles, NULL, BLE_CONFIG_NUM_ADV_SETS);
            break;
        }

        default:
        {
            break;
        }
    }
}

/*********************************************************************
 * @fn      Peripheral_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the peripheral
 *          application module
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t Peripheral_start()
{
    bStatus_t status = SUCCESS;

    status = BLEAppUtil_registerEventHandler(&peripheralConnHandler);
    if(status != SUCCESS)
    {
        // Return status value
        return(status);
    }

    status = BLEAppUtil_registerEventHandler(&peripheralAdvHandler);
    if(status != SUCCESS)
    {
        return(status);
    }

    // Initiate the advertise sets
    uint8_t peripheralAdvHandlesStatuses[BLE_CONFIG_NUM_ADV_SETS];
    BleConfig_initAdvSets(peripheralAdvHandles, peripheralAdvHandlesStatuses);
    for (int i = 0; i < BLE_CONFIG_NUM_ADV_SETS; i++)
    {
        // Return the first FAILURE status value
        if (peripheralAdvHandlesStatuses[i] != SUCCESS)
        {
            return(peripheralAdvHandlesStatuses[i]);
        }
    }
    
    BleConfig_startAdvSets(peripheralAdvHandles, NULL, BLE_CONFIG_NUM_ADV_SETS);
    //! UNCOMMENT HERE
    // App_initParamUpdateClock();

    // Return status value
    return(status);
}

#endif // ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
