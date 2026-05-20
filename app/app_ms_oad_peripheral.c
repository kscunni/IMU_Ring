/******************************************************************************

@file  app_ms_oad_peripheral.c

@brief This file demonstrates how to activate the peripheral role with
the help of BLEAppUtil APIs for Multi-Step OAD applications.

This is a simplified version of app_peripheral.c without menu/printing
functionality, designed for OAD operations where minimal code footprint
is desired.

Two structures are used for event handling, one for connection events and one
for advertising events.
In each, eventMask is used to specify the events that will be received
and handled.
In addition, fill the BLEAppUtil_AdvInit_t structure with variables generated
by the Sysconfig.

In the events handler functions, write what actions are done after each event.
In this example, after a connection is made, activation is performed for
re-advertising up to the maximum connections.

In the MsOadPeripheral_start() function at the bottom of the file, registration,
initialization and activation are done using the BLEAppUtil API functions,
using the structures defined in the file.

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

#ifdef MS_OAD

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti_ble_config.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_main.h>

//*****************************************************************************
//! Prototypes
//*****************************************************************************
void MsOadPeripheral_AdvEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
void MsOadPeripheral_GAPConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************

BLEAppUtil_EventHandler_t msOadPeripheralConnHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_CONN_TYPE,
    .pEventHandler  = MsOadPeripheral_GAPConnEventHandler,
    .eventMask      = BLEAPPUTIL_LINK_ESTABLISHED_EVENT |
                      BLEAPPUTIL_LINK_PARAM_UPDATE_REQ_EVENT |
                      BLEAPPUTIL_LINK_TERMINATED_EVENT,
};

BLEAppUtil_EventHandler_t msOadPeripheralAdvHandler =
{
    .handlerType    = BLEAPPUTIL_GAP_ADV_TYPE,
    .pEventHandler  = MsOadPeripheral_AdvEventHandler,
    .eventMask      = BLEAPPUTIL_ADV_START_AFTER_ENABLE |
                      BLEAPPUTIL_ADV_END_AFTER_DISABLE
};

//! Stores adv handles
uint8_t msOadPeripheralAdvHandles[BLE_CONFIG_NUM_ADV_SETS];

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      MsOadPeripheral_AdvEventHandler
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
void MsOadPeripheral_AdvEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_ADV_START_AFTER_ENABLE:
        {
            // Advertising started - no action needed for MS-OAD
            break;
        }

        case BLEAPPUTIL_ADV_END_AFTER_DISABLE:
        {
            // Advertising ended - no action needed for MS-OAD
            break;
        }

        default:
        {
            break;
        }
    }
}

/*********************************************************************
 * @fn      MsOadPeripheral_GAPConnEventHandler
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
void MsOadPeripheral_GAPConnEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch(event)
    {
        case BLEAPPUTIL_LINK_ESTABLISHED_EVENT:
        {
            /* Check if we reach the maximum allowed number of connections */
            if(linkDB_NumActive() < linkDB_NumConns())
            {
                /* Start advertising since there is room for more connections */
                BleConfig_startAdvSets(msOadPeripheralAdvHandles, NULL, BLE_CONFIG_NUM_ADV_SETS);
            }
            else
            {
                /* Stop advertising since there is no room for more connections */
                BleConfig_stopAdvSets(msOadPeripheralAdvHandles, NULL, BLE_CONFIG_NUM_ADV_SETS);
            }
            break;
        }

        case BLEAPPUTIL_LINK_TERMINATED_EVENT:
        {
            BleConfig_startAdvSets(msOadPeripheralAdvHandles, NULL, BLE_CONFIG_NUM_ADV_SETS);
            break;
        }

        case BLEAPPUTIL_LINK_PARAM_UPDATE_REQ_EVENT:
        {
            gapUpdateLinkParamReqEvent_t *pReq = (gapUpdateLinkParamReqEvent_t *)pMsgData;
            // Accept all parameter update requests for OAD
            BLEAppUtil_paramUpdateRsp(pReq, TRUE);
            break;
        }

        default:
        {
            break;
        }
    }
}

/*********************************************************************
 * @fn      MsOadPeripheral_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the peripheral
 *          application module for Multi-Step OAD
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t MsOadPeripheral_start(void)
{
    bStatus_t status = SUCCESS;

    status = BLEAppUtil_registerEventHandler(&msOadPeripheralConnHandler);
    if(status != SUCCESS)
    {
        // Return status value
        return(status);
    }

    status = BLEAppUtil_registerEventHandler(&msOadPeripheralAdvHandler);
    if(status != SUCCESS)
    {
        return(status);
    }

    // Initiate the advertise sets
    uint8_t msOadPeripheralAdvHandlesStatuses[BLE_CONFIG_NUM_ADV_SETS];
    BleConfig_initAdvSets(msOadPeripheralAdvHandles, msOadPeripheralAdvHandlesStatuses);
    for (int i = 0; i < BLE_CONFIG_NUM_ADV_SETS; i++)
    {
        // Return the first FAILURE status value
        if (msOadPeripheralAdvHandlesStatuses[i] != SUCCESS)
        {
            return(msOadPeripheralAdvHandlesStatuses[i]);
        }
    }

    BleConfig_startAdvSets(msOadPeripheralAdvHandles, NULL, BLE_CONFIG_NUM_ADV_SETS);

    // Return status value
    return(status);
}

#endif // MS_OAD
