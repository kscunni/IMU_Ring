/******************************************************************************

@file  app_padv_time_sync.h

@brief This file contains the Periodic Advertising Time Synchronization module
       for BLE applications.

The app_padv_time_sync module provides time synchronization using BLE periodic
advertising. It supports two roles:

Time Sync Advertiser (TSA):
- Creates periodic advertising to broadcast the local system time
- Acts as the time reference for the network

Time Sync Observer (TSO):
- Listens for periodic advertising from a TSA
- Receives PADV events with absStartTime from the stack
- Updates app_time_sync with the synchronized time reference

Usage:
1. Create a configuration structure with role and parameters
2. Call AppPadvTimeSync_init() with the configuration
3. For TSO: Time sync happens automatically via callbacks
4. For TSA: Periodic advertising broadcasts the time

This module uses BLEAppUtil as an abstraction layer and does not
call GAP APIs directly.

Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2024-2026, Texas Instruments Incorporated
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

#ifndef APP_PADV_TIME_SYNC_H
#define APP_PADV_TIME_SYNC_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <stdint.h>
#include "ti/ble/stack_util/bcomdef.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"

//*****************************************************************************
//! Defines
//*****************************************************************************

// Default values that can be overridden by SysConfig
// When SysConfig is used, these values are defined via compiler flags

/// Default periodic advertising interval minimum (400 * 1.25ms = 500ms)
#ifndef APP_PADV_TIME_SYNC_PERIODIC_ADV_INTERVAL_MIN
#define APP_PADV_TIME_SYNC_PERIODIC_ADV_INTERVAL_MIN    400
#endif

/// Default periodic advertising interval maximum (400 * 1.25ms = 500ms)
#ifndef APP_PADV_TIME_SYNC_PERIODIC_ADV_INTERVAL_MAX
#define APP_PADV_TIME_SYNC_PERIODIC_ADV_INTERVAL_MAX    400
#endif

/// Default sync timeout (200 * 10ms = 2s)
#ifndef APP_PADV_TIME_SYNC_TIMEOUT
#define APP_PADV_TIME_SYNC_TIMEOUT                      200
#endif

/// Default skip value (no events skipped)
#ifndef APP_PADV_TIME_SYNC_SKIP
#define APP_PADV_TIME_SYNC_SKIP                         0
#endif

/// Default advertising SID for time sync
#ifndef APP_PADV_TIME_SYNC_ADV_SID
#define APP_PADV_TIME_SYNC_ADV_SID                      1
#endif

#define APP_PADV_TIME_SYNC_INTERVAL                     160
#define APP_PADV_TIME_SYNC_SCAN_WINDOW                  160

//*****************************************************************************
//! Types
//*****************************************************************************

/**
 * @brief Time Sync Role
 *
 * Defines the role of the device in the time synchronization network.
 */
typedef enum
{
    APP_PADV_TIME_SYNC_ROLE_TSA,  ///< Time Sync Advertiser - broadcasts local time
    APP_PADV_TIME_SYNC_ROLE_TSO   ///< Time Sync Observer - listens for time updates
} AppPadvTimeSync_Role_t;

/**
 * @brief TSO State Machine States
 *
 * Defines the state of the Time Sync Observer during TSA discovery and sync.
 */
typedef enum
{
    APP_PADV_TIME_SYNC_TSO_STATE_IDLE,      ///< Initial state, not scanning or synced
    APP_PADV_TIME_SYNC_TSO_STATE_SCANNING,  ///< Scanning for TSA by name
    APP_PADV_TIME_SYNC_TSO_STATE_SYNCING,   ///< TSA found, creating periodic sync
    APP_PADV_TIME_SYNC_TSO_STATE_SYNCED     ///< Periodic sync established
} AppPadvTimeSync_TsoState_t;

/**
 * @brief Time Sync Configuration for TSA (Time Sync Advertiser)
 */
typedef struct
{
    const char *tsaName;               ///< TSA device name to advertise (required for TSO discovery)
    uint8_t advSID;                    ///< Advertising SID to use (0-15)
    uint16_t periodicAdvIntervalMin;   ///< Min periodic adv interval (N * 1.25ms)
    uint16_t periodicAdvIntervalMax;   ///< Max periodic adv interval (N * 1.25ms)
    uint16_t periodicAdvProp;          ///< Periodic adv properties (bit 6: TxPower)
} AppPadvTimeSync_TsaConfig_t;

/**
 * @brief Time Sync Configuration for TSO (Time Sync Observer)
 */
typedef struct
{
    const char *tsaName;               ///< Name of TSA to find (required, cannot be NULL)
    uint8_t  advSID;                   ///< Advertising SID to sync with (0-15)
    uint16_t skip;                     ///< Max events to skip (0x0000 to 0x01F3)
    uint16_t syncTimeout;              ///< Sync timeout in 10ms units (0x000A to 0x4000)
    uint8_t  options;                  ///< Sync options (bit 0: use list, bit 1: reporting)
} AppPadvTimeSync_TsoConfig_t;

/**
 * @brief Time Sync Configuration
 *
 * Union of role-specific configurations.
 */
typedef struct
{
    AppPadvTimeSync_Role_t role;       ///< Device role (TSA or TSO)
    union
    {
        AppPadvTimeSync_TsaConfig_t tsa;  ///< TSA configuration
        AppPadvTimeSync_TsoConfig_t tso;  ///< TSO configuration
    } config;
} AppPadvTimeSync_Config_t;

//*****************************************************************************
//! Functions
//*****************************************************************************

/**
 * @brief  Initialize the PADV Time Sync module
 *
 * Initializes the module based on the specified role:
 *
 * For TSA (Time Sync Advertiser):
 * - Creates an extended advertising set
 * - Configures periodic advertising parameters
 * - Enables periodic advertising to broadcast time
 *
 * For TSO (Time Sync Observer):
 * - Registers the PADV event callback
 * - Creates periodic advertising sync to receive time updates
 *
 * @param  pConfig - Pointer to configuration structure containing role and
 *                   role-specific parameters. Must not be NULL.
 *
 * @return SUCCESS if initialization successful
 * @return INVALIDPARAMETER if pConfig is NULL or contains invalid values
 * @return bleNoResources if resources could not be allocated
 */
uint8_t AppPadvTimeSync_init(AppPadvTimeSync_Config_t *pConfig);

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG ) )
/*********************************************************************
 * @fn      AppPadvTimeSync_start
 *
 * @brief   Save syncHandle to LL to send to application the expected data
 * 
 * @param  syncHandle - Handle identifying the periodic advertising train to sync with
 *
 * @return  SUCCESS if started successful
 */
uint8_t AppPadvTimeSync_start(uint8_t syncHandle);

/**
 * @brief  Scan Event Handler for TSO name-based discovery
 *
 * Handles GAP scan events during TSA discovery. This function processes
 * advertisement reports to find the TSA device by name, then initiates
 * periodic advertising sync.
 *
 * @note   This handler is registered internally during TSO initialization
 *         when name-based discovery is enabled (tsaName != NULL).
 *
 * @param  event    - GAP scan event type
 * @param  pMsgData - Pointer to event-specific data
 *
 * @return None
 */
void AppPadvTimeSync_ScanEventHandler(uint32 event, BLEAppUtil_msgHdr_t *pMsgData);
#endif // HOST_CONFIG & ( CENTRAL_CFG | OBSERVER_CFG )

#ifdef __cplusplus
}
#endif

#endif /* APP_PADV_TIME_SYNC_H */
