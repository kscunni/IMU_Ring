/******************************************************************************

@file  app_time_sync.h

@brief This file contains the Time Synchronization module for BLE applications.

The app_time_sync module provides time synchronization capabilities using
BLE Periodic Advertising. It maintains an offset between the local system
clock and an absolute time reference shared across devices.

The module supports two roles (configured via APP_TIME_SYNC_ROLE):
- TSA (Time Sync Advertiser): Broadcasts local time via periodic advertising
- TSO (Time Sync Observer): Syncs to TSA's time via periodic scanning

Configuration is done through compile-time defines (APP_TIME_SYNC_*) that
can be set via SysConfig. The module internally uses app_padv_time_sync
to handle BLE-specific operations.

Usage:
1. Configure APP_TIME_SYNC_* defines (via SysConfig or manually)
2. Call AppTimeSync_init() to initialize the module
3. Use AppTimeSync_getCurrentSharedTime() to query synchronized time

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

#ifndef APP_TIME_SYNC_H
#define APP_TIME_SYNC_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "app_padv_time_sync.h"

//*****************************************************************************
//! SysConfig Overridable Defines
//*****************************************************************************
// These defaults can be overridden by SysConfig-generated defines

#ifndef APP_TIME_SYNC_ROLE
#define APP_TIME_SYNC_ROLE                      APP_PADV_TIME_SYNC_ROLE_TSO
#endif

// TSA Configuration Defaults
#ifndef APP_TIME_SYNC_TSA_PERIODIC_ADV_INTERVAL_MIN
#define APP_TIME_SYNC_TSA_PERIODIC_ADV_INTERVAL_MIN   APP_PADV_TIME_SYNC_PERIODIC_ADV_INTERVAL_MIN
#endif

#ifndef APP_TIME_SYNC_TSA_PERIODIC_ADV_INTERVAL_MAX
#define APP_TIME_SYNC_TSA_PERIODIC_ADV_INTERVAL_MAX   APP_PADV_TIME_SYNC_PERIODIC_ADV_INTERVAL_MAX
#endif

#ifndef APP_TIME_SYNC_TSA_PERIODIC_ADV_PROP
#define APP_TIME_SYNC_TSA_PERIODIC_ADV_PROP           0
#endif

// TSO Configuration Defaults
#ifndef APP_TIME_SYNC_ADV_SID
#define APP_TIME_SYNC_ADV_SID                     APP_PADV_TIME_SYNC_ADV_SID
#endif

#ifndef APP_TIME_SYNC_TSO_SKIP
#define APP_TIME_SYNC_TSO_SKIP                        APP_PADV_TIME_SYNC_SKIP
#endif

#ifndef APP_TIME_SYNC_TSO_SYNC_TIMEOUT
#define APP_TIME_SYNC_TSO_SYNC_TIMEOUT                APP_PADV_TIME_SYNC_TIMEOUT
#endif

#ifndef APP_TIME_SYNC_TSO_OPTIONS
#define APP_TIME_SYNC_TSO_OPTIONS                     0
#endif

//*****************************************************************************
//! Typedefs
//*****************************************************************************

/**
 * @brief Time sync module state structure
 */
typedef struct
{
    uint32_t setupTimeOffset;     /**< Offset between local and absolute time (ms) */
    uint32_t lastSyncTime;        /**< Local time of last setAbsTime() call (ms) */
    bool     initialized;         /**< True if module has been initialized */
} AppTimeSync_state_t;


//*****************************************************************************
//! Functions
//*****************************************************************************

/**
 * @brief  Initialize the time sync module
 *
 * Initializes the time sync module with optional external time source support.
 * The behavior depends on the callback parameter:
 *
 * - If cb is NULL: Uses the built-in PADV time source. The module role (TSA or
 *   TSO) and configuration parameters are determined by compile-time defines
 *   that can be set via SysConfig. Internally calls AppPadvTimeSync_init().
 *
 * - If cb is NOT NULL: Uses an external time source. The callback receives
 *   a pointer to AppTimeSync_setTimeOffset, allowing the external module to
 *   update the absolute time reference. PADV is not initialized in this mode.
 *
 * In both cases, the time sync state is initialized (offset = 0, initialized = true).
 *
 * Configuration for PADV mode is controlled by APP_TIME_SYNC_* defines:
 * - APP_TIME_SYNC_ROLE: TSA or TSO role
 * - APP_TIME_SYNC_TSA_*: TSA-specific parameters
 * - APP_TIME_SYNC_TSO_*: TSO-specific parameters
 *
 * @design REQ-1
 * @trace  AC-2
 *
 * @param  usePadv - Flag to use PADV or external sync source
 *
 * @return SUCCESS if initialization successful
 * @return Error code if PADV initialization failed (only when cb is NULL)
 */
uint8_t AppTimeSync_init(uint8_t usePadv);

/**
 * @fn      AppTimeSync_setTimeOffset
 *
 * @brief   Set the offset from the absolute time reference in the setup.
 *          This function should be called:
 *              - TSO: When receiving time from TSA (via PADV).
 *              - TSA: When setting the initial system-wide reference time.
 *
 * @param   offset - The absolute offset time in from the time source (in ms)
 *
 * @return  none
 */
void AppTimeSync_setTimeOffset(uint32_t offset);

/**
 * @brief  Get the current absolute time
 *
 * Returns the current absolute time by adding the stored offset
 * to the local time. Handles 32-bit wraparound correctly using
 * unsigned modular arithmetic.
 *
 * Formula: absTime = BLEAppUtil_getCurrentTime() + offset
 *
 * @design REQ-3, REQ-5
 * @trace  AC-4, AC-6
 *
 * @return Current absolute time in milliseconds
 */
uint32_t AppTimeSync_getCurrentSharedTime(void);

/**
 * @brief  Get offset time since a start point
 *
 * Calculates the offset time between the current absolute time
 * and a given start time. Handles 32-bit wraparound correctly.
 *
 * Formula: offset = AppTimeSync_getCurrentSharedTime() - startTime
 *
 * @design REQ-4, REQ-5
 * @trace  AC-5, AC-6
 *
 * @param  startTime - The start time to calculate offset from (in ms)
 *
 * @return Offset time in milliseconds since startTime
 */
uint32_t AppTimeSync_getTimeDeltaInUs(uint32_t startTime);

/**
 * @brief  Get the stored time offset
 *
 * Returns the raw offset value for debugging or diagnostics.
 *
 * @return The stored offset in milliseconds
 */
uint32_t AppTimeSync_getCurrentSharedTime(void);

/**
 * @brief  Get the last time sync was set, in local time.
 *
 * Returns the local time of the last setAbsTime() call.
 *
 * @return The last sync time in milliseconds
 */
uint32_t AppTimeSync_getLastSyncTime(void);

/*********************************************************************
 * @fn      AppTimeSync_isEnabled
 *
 * @brief  Check if time synchronization is enabled.
 *
 * @return TRUE if time synchronization is enabled, FALSE otherwise
 */
bool AppTimeSync_isEnabled(void);

/*********************************************************************
 * @fn      AppTimeSync_getTimeSyncSize
 *
 * @brief  Get the size of the time sync value (4 bytes)
 *
 * @return The size of the time sync value in bytes
 */
uint32_t AppTimeSync_getTimeSyncSize(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TIME_SYNC_H */
