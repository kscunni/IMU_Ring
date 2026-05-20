/******************************************************************************

@file  app_time_sync.c

@brief This file contains the Time Synchronization module implementation.

The app_time_sync module provides time synchronization using BLE Periodic
Advertising. It maintains an offset between the local system clock and an
absolute time reference shared across devices.

Architecture:
- Internally uses app_padv_time_sync for BLE-specific operations
- Configuration via compile-time defines (APP_TIME_SYNC_*) set by SysConfig
- TSA role: Creates periodic advertising to broadcast time
- TSO role: Syncs to TSA's time via periodic scanning

Time Sync Concept:
- TSA broadcasts its local time as the absolute reference
- TSO receives time and calculates offset: offset = absTime - localTime
- Applications query time via AppTimeSync_getCurrentSharedTime()
- Periodic updates (~500ms) correct for clock drift
- All calculations handle 32-bit wraparound using unsigned modular arithmetic

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

#ifdef TIME_SYNC

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "app_time_sync.h"
#include "app_padv_time_sync.h"
#include "ti_ble_config.h"
#include "ti/ble/stack_util/bcomdef.h"
#include "ti/ble/app_util/framework/bleapputil_api.h"

//*****************************************************************************
//! Local Variables
//*****************************************************************************

/**
 * @brief Module state - stores offset and initialization status
 */
static AppTimeSync_state_t appTimeSyncState = {0};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      AppTimeSync_init
 *
 * @brief   Initialize the time sync module
 *
 *          Initializes the time sync module with optional external time
 *          source support. If cb is NULL, uses the built-in PADV time
 *          source. If cb is provided, passes the setAbsTime function
 *          pointer to the callback for external time source integration.
 *
 * @design  REQ-1
 * @trace   AC-2
 *
 * @param   cb - Callback to provide AppTimeSync_setTimeOffset to external time source, or NULL to use PADV
 *
 * @return  SUCCESS if successful, error code otherwise
 */
uint8_t AppTimeSync_init(uint8_t usePadv)
{
    uint8_t status = SUCCESS;
    // Initialize state to defaults
    appTimeSyncState.setupTimeOffset = 0;
    appTimeSyncState.lastSyncTime = 0;
    appTimeSyncState.initialized = TRUE;

    if (usePadv == TRUE)
    {
        // Use PADV implementation (default behavior)
        AppPadvTimeSync_Config_t padvConfig;

        // Configure PADV time sync based on role
        padvConfig.role = APP_TIME_SYNC_ROLE;

        if (APP_TIME_SYNC_ROLE == APP_PADV_TIME_SYNC_ROLE_TSA)
        {
            // TSA configuration
            padvConfig.config.tsa.tsaName = APP_PADV_TIME_SYNC_TSA_NAME;
            padvConfig.config.tsa.advSID = APP_TIME_SYNC_ADV_SID;
            padvConfig.config.tsa.periodicAdvIntervalMin = APP_TIME_SYNC_TSA_PERIODIC_ADV_INTERVAL_MIN;
            padvConfig.config.tsa.periodicAdvIntervalMax = APP_TIME_SYNC_TSA_PERIODIC_ADV_INTERVAL_MAX;
            padvConfig.config.tsa.periodicAdvProp = APP_TIME_SYNC_TSA_PERIODIC_ADV_PROP;
        }
        else
        {
            // TSO configuration
            padvConfig.config.tso.tsaName = APP_PADV_TIME_SYNC_TSA_NAME;
            padvConfig.config.tso.advSID = APP_TIME_SYNC_ADV_SID;
            padvConfig.config.tso.skip = APP_TIME_SYNC_TSO_SKIP;
            padvConfig.config.tso.syncTimeout = APP_TIME_SYNC_TSO_SYNC_TIMEOUT;
            padvConfig.config.tso.options = APP_TIME_SYNC_TSO_OPTIONS;
        }

        // Initialize PADV time sync
        status = AppPadvTimeSync_init(&padvConfig);
    }

    return status;
}

/*********************************************************************
 * @fn      AppTimeSync_setTimeOffset
 *
 * @brief   Set the offset from the absolute time reference in the setup.
 *
 *
 * @param   offset - The absolute offset time in from the time source (in ms)
 *
 * @return  none
 */
void AppTimeSync_setTimeOffset(uint32_t offset)
{
    // Calculate offset using unsigned arithmetic
    // This works correctly even when absTime < localTime due to modular math
    appTimeSyncState.setupTimeOffset = offset;

    // Record the sync time for drift analysis
    appTimeSyncState.lastSyncTime = BLEAppUtil_getCurrentTime() / 4;
}

/*********************************************************************
 * @fn      AppTimeSync_getCurrentSharedTime
 *
 * @brief   Get the current shared time in the synchronized time domain
 *
 *          Returns the current shared time by adding the stored offset
 *          to the local time. Unsigned arithmetic handles 32-bit wraparound
 *          correctly.
 *
 *          Formula: sharedTime = localTime + offset
 *
 * @design  REQ-3, REQ-5
 * @trace   AC-4, AC-6
 *
 * @return  Current shared time in milliseconds
 */
uint32_t AppTimeSync_getCurrentSharedTime(void)
{
    uint32_t sharedTime;

    // Get current local time
    sharedTime = BLEAppUtil_getCurrentTime() / 4;

    // Calculate shared time using unsigned arithmetic ("normalization" of local time)
    // Handles 32-bit wraparound correctly
    if (APP_TIME_SYNC_ROLE == APP_PADV_TIME_SYNC_ROLE_TSO)
    {
        sharedTime += appTimeSyncState.setupTimeOffset;
    }

    return sharedTime;
}

/*********************************************************************
 * @fn      AppTimeSync_getTimeDeltaInUs
 *
 * @brief   Get offset time since a start point
 *
 *          Calculates the offset time between the current absolute time
 *          and a given start time. Unsigned subtraction handles 32-bit
 *          wraparound correctly.
 *
 *          Formula: timeDelta = getCurrentSharedTime() - startTime
 *
 * @design  REQ-4, REQ-5
 * @trace   AC-5, AC-6
 *
 * @param   startTime - The start time to calculate offset from (in ms)
 *
 * @return  Elapsed time in milliseconds since startTime
 */
uint32_t AppTimeSync_getTimeDeltaInUs(uint32_t startTime)
{
    uint32_t currentAbsTime;
    uint32_t timeDelta;

    // Get current shared time
    currentAbsTime = AppTimeSync_getCurrentSharedTime();

    // Calculate elapsed time using unsigned arithmetic
    // Handles 32-bit wraparound correctly (REQ-5, AC-6)
    timeDelta = (currentAbsTime - startTime);

    return timeDelta;
}

/*********************************************************************
 * @fn      AppTimeSync_getSharedTimeOffset
 *
 * @brief   Get the stored time offset
 *
 *          Returns the raw offset value for debugging or diagnostics.
 *
 * @return  The stored offset in milliseconds
 */
uint32_t AppTimeSync_getSharedTimeOffset(void)
{
    return appTimeSyncState.setupTimeOffset;
}

/*********************************************************************
 * @fn      AppTimeSync_getLastSyncTime
 *
 * @brief  Get the last time sync was set, in local time.
 *
 * Returns the local time of the last setAbsTime() call.
 *
 * @return The last sync time in milliseconds
 */
uint32_t AppTimeSync_getLastSyncTime(void)
{
    return appTimeSyncState.lastSyncTime;
}

/*********************************************************************
 * @fn      AppTimeSync_isEnabled
 *
 * @brief  Check if time synchronization is enabled.
 *
 * @return TRUE if time synchronization is enabled, FALSE otherwise
 */
bool AppTimeSync_isEnabled(void)
{
    return appTimeSyncState.initialized;
}

/*********************************************************************
 * @fn      AppTimeSync_getTimeSyncSize
 *
 * @brief  Get the size of the time sync value (4 bytes)
 *
 * @return The size of the time sync value in bytes
 */
uint32_t AppTimeSync_getTimeSyncSize(void)
{
    return sizeof(appTimeSyncState.setupTimeOffset);
}

#endif // TIME_SYNC