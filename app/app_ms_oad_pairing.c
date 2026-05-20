/******************************************************************************

@file  app_ms_oad_pairing.c

@brief This file contains the application pairing role functionality
for Multi-Step OAD applications.

This is a simplified version of app_pairing.c without menu/printing
functionality, designed for OAD operations where minimal code footprint
is desired.

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
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include <app_main.h>
#include <ti/log/Log.h>

//*****************************************************************************
//! Prototypes
//*****************************************************************************

void MsOadPairing_passcodeHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
void MsOadPairing_pairStateHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);

//*****************************************************************************
//! Globals
//*****************************************************************************

BLEAppUtil_EventHandler_t msOadPairingPasscodeHandler =
{
    .handlerType    = BLEAPPUTIL_PASSCODE_TYPE,
    .pEventHandler  = MsOadPairing_passcodeHandler
};

BLEAppUtil_EventHandler_t msOadPairingPairStateHandler =
{
    .handlerType    = BLEAPPUTIL_PAIR_STATE_TYPE,
    .pEventHandler  = MsOadPairing_pairStateHandler,
    .eventMask      = BLEAPPUTIL_PAIRING_STATE_STARTED |
                      BLEAPPUTIL_PAIRING_STATE_COMPLETE |
                      BLEAPPUTIL_PAIRING_STATE_ENCRYPTED |
                      BLEAPPUTIL_PAIRING_STATE_BOND_SAVED,
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      MsOadPairing_passcodeHandler
 *
 * @brief   The purpose of this function is to handle passcode data
 *          that rise from the GAPBondMgr and were registered
 *          in @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void MsOadPairing_passcodeHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    BLEAppUtil_PasscodeData_t *pData = (BLEAppUtil_PasscodeData_t *)pMsgData;

    // Send passcode response
    // TODO: Application should use actual passcode handling logic
    GAPBondMgr_PasscodeRsp(pData->connHandle, SUCCESS, B_APP_DEFAULT_PASSCODE);
}

/*********************************************************************
 * @fn      MsOadPairing_pairStateHandler
 *
 * @brief   The purpose of this function is to handle pairing state
 *          events that rise from the GAPBondMgr and were registered
 *          in @ref BLEAppUtil_RegisterGAPEvent
 *
 * @param   event - message event.
 * @param   pMsgData - pointer to message data.
 *
 * @return  none
 */
void MsOadPairing_pairStateHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData)
{
    switch (event)
    {
        case BLEAPPUTIL_PAIRING_STATE_STARTED:
        {
            // Pairing started - no action needed for MS-OAD
            Log_printf(LogModule_OAD_MS, Log_INFO, "MS_OAD: MsOadPairing_pairStateHandler(): Pairing started");
            break;
        }
        case BLEAPPUTIL_PAIRING_STATE_COMPLETE:
        {
            // The pairing is completed
            Log_printf(LogModule_OAD_MS, Log_INFO, "MS_OAD: MsOadPairing_pairStateHandler(): Pairing completed");
            break;
        }

        case BLEAPPUTIL_PAIRING_STATE_ENCRYPTED:
        {
            // Link encrypted
            // TODO: Handle link encryption event
            Log_printf(LogModule_OAD_MS, Log_INFO, "MS_OAD: MsOadPairing_pairStateHandler(): Link encrypted");
            break;
        }

        case BLEAPPUTIL_PAIRING_STATE_BOND_SAVED:
        {
            // Bond saved
            // TODO: Handle bond saved event
            Log_printf(LogModule_OAD_MS, Log_INFO, "MS_OAD: MsOadPairing_pairStateHandler(): Bond saved");
            break;
        }

        default:
        {
            break;
        }
    }

}

/*********************************************************************
 * @fn      MsOadPairing_start
 *
 * @brief   This function is called after stack initialization,
 *          the purpose of this function is to initialize and
 *          register the specific events handlers of the pairing
 *          application module for Multi-Step OAD
 *
 * @return  SUCCESS, errorInfo
 */
bStatus_t MsOadPairing_start(void)
{
    bStatus_t status = SUCCESS;

    // Register the handlers
    status = BLEAppUtil_registerEventHandler(&msOadPairingPasscodeHandler);
    if (status != SUCCESS)
    {
        return(status);
    }

    status = BLEAppUtil_registerEventHandler(&msOadPairingPairStateHandler);
    if (status != SUCCESS)
    {
        // Return status value
        return(status);
    }

    // Return status value
    return(status);
}

#endif // MS_OAD
