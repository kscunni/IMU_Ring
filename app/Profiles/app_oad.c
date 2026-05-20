/******************************************************************************

@file  app_dev_info.c

@brief This file contains the device info application functionality

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

#if defined (OAD_CFG) && !defined(OAD_PERSISTENT)

//*****************************************************************************
//! Includes
//*****************************************************************************
#include <string.h>
#include "ti/ble/app_util/framework/bleapputil_api.h"
#include "ti/ble/app_util/menu/menu_module.h"
#include <app_main.h>
#include "ti/ble/profiles/oad/oad_profile.h"
#include "ti/ble/app_util/sw_update/sw_update.h"
#include "ti/ble/profiles/oad/ms_oad_profile.h"

//*****************************************************************************
//! Defines
//*****************************************************************************

/**
 * @defgroup APP_OAD_States Application OAD State Flags
 * @brief Bitmap flags for tracking OAD profile state in the application layer
 *
 * These flags are used in conjunction with AppMsOadData_t.oadFlags to track
 * the progress of OAD operations at the application level.
 * @{
 */

/** Application is idle, no OAD operation in progress */
#define APP_OAD_STATE_IDLE                ((uint8_t) BV(0))

/** New image has been identified and validated during OAD_PROFILE_MSG_NEW_IMG_IDENTIFY */
#define APP_OAD_STATE_IMAGE_IDENTIFIED    ((uint8_t) BV(1))

/** @} */ // end of APP_OAD_States

//*****************************************************************************
//! Typedefs
//*****************************************************************************

#ifdef MS_OAD
/**
 * @brief Application-level MS-OAD data structure
 *
 * This structure maintains the application's view of the MS-OAD state and
 * tracks the OAD operation progress. It serves as the interface between the
 * MS-OAD profile and the application logic.
 *
 * @note The msOadState field reflects the MS-OAD module's public state, while
 *       oadFlags tracks application-specific state transitions during OAD operations.
 */
typedef struct
{
    /**
     * Public state of MS-OAD module, determined during MS-OAD initialization.
     * This reflects the bootloader's view of the system state (e.g., whether
     * a new image is ready for verification, or if the system is idle).
     * Retrieved via MSOAD_GetState().
     * @see msOadState_e
     */
    msOadState_e  msOadState;

    /**
     * Bitmap of application-level OAD state flags.
     * Tracks the progress of OAD operations within the application layer.
     * Uses flags defined in @ref APP_OAD_States (e.g., APP_OAD_STATE_IMAGE_IDENTIFIED).
     */
    uint8_t oadFlags;

    /**
     * Version of the new image identified during the OAD identify phase.
     * Saved when OAD_PROFILE_MSG_NEW_IMG_IDENTIFY is received and validation succeeds.
     * Used later during MS-OAD initialization (MSOAD_InitAndReset()) to inform the
     * bootloader which version to expect.
     * @see OAD_PROFILE_MSG_NEW_IMG_IDENTIFY
     */
    struct image_version identifiedImageVersion;
} AppMsOadData_t;
#endif

//*****************************************************************************
//! Globals
//*****************************************************************************

#ifdef MS_OAD
/**
 * @brief Global application MS-OAD data structure
 *
 * Maintains the current state of MS-OAD operations from the application's perspective.
 * Initialized with default values and updated throughout the OAD process lifecycle:
 * - msOadState: Updated in OAD_start() via MSOAD_GetState()
 * - oadFlags: Updated in App_OADCallbackMsOad() as OAD progresses
 * - identifiedImageVersion: Saved during OAD_PROFILE_MSG_NEW_IMG_IDENTIFY
 *
 * @note This structure is reset to idle state when OAD_start() is called on device boot.
 */
AppMsOadData_t gAppMsOadData = {
    .msOadState = MS_OAD_UNINITIALIZED,
    .oadFlags = APP_OAD_STATE_IDLE,
    .identifiedImageVersion = {0,0,0,0}
};
#endif

//*****************************************************************************
//! Functions
//*****************************************************************************

/**
 * @brief Validate new OAD image version
 *
 * Application-specific validation function called during the OAD image
 * identification phase (OAD_PROFILE_MSG_NEW_IMG_IDENTIFY). This function
 * allows the application to enforce version policies before accepting a
 * new image for download.
 *
 * @param imageHeader Pointer to the image header of the candidate OAD image.
 *                    Contains version information (ih_ver) and other metadata.
 *
 * @return true if the new image version is acceptable and OAD should proceed
 * @return false if the image should be rejected (e.g., version downgrade not allowed)
 *
 * @note **Current implementation always returns true (no validation).**
 *       Applications should customize this function to implement their version
 *       policy. Common checks include:
 *       - Enforce version must be higher than current version
 *       - Allow/disallow downgrades
 *       - Check for specific version ranges
 *       - Validate version format
 *
 * @warning This is a placeholder implementation. Production code should
 *          implement proper version validation logic.
 */
bool App_validateNewVersion(struct image_header *imageHeader)
{
    VOID imageHeader;

    // TODO: Application should check if the new image version is acceptable.
    //       For example, check if version is higher than current version, etc.

    return true;
}

#ifdef MS_OAD
/**
 * @brief OAD Profile callback for Multistep-OAD (MS-OAD) operations
 *
 * Application callback function invoked by the OAD Profile during MS-OAD operations.
 * This callback handles MS-OAD-specific events and enforces application-level policies
 * for the OAD process. Unlike the standard OAD callback (@ref App_OADCallback), this
 * callback manages the multistep nature of MS-OAD, including initialization, state
 * tracking, and image verification.
 *
 * @param msg OAD Profile event/message type. Handled events:
 *            - OAD_PROFILE_MSG_NEW_IMG_IDENTIFY: New image identification request
 *            - OAD_PROFILE_MSG_GET_MS_OAD_STATE: Query current MS-OAD state
 *            - OAD_PROFILE_MSG_MS_OAD_INIT_REQ: MS-OAD initialization request
 *            - OAD_PROFILE_MSG_MS_OAD_COMMIT: Image verification/commit request
 *            All other OAD profile events are rejected (OAD_PROFILE_CANCEL)
 *
 * @param pData Context-specific data pointer:
 *              - NEW_IMG_IDENTIFY: Points to struct image_header of candidate image
 *              - GET_MS_OAD_STATE: Output pointer for uint8 state value
 *              - MS_OAD_INIT_REQ: Not used
 *              - MS_OAD_COMMIT: Not used
 *
 * @return OAD_PROFILE_PROCEED to allow the operation to continue
 * @return OAD_PROFILE_CANCEL to reject the operation
 *
 * @note This callback works in conjunction with @ref gAppMsOadData to track
 *       application-level OAD state across the multistep process.
 *
 * @see App_OADCallback for standard (non-multistep) OAD operations
 * @see msOadState_e for MS-OAD state definitions
 * @see MSOAD_InitAndReset for MS-OAD initialization
 * @see MSOAD_ImageVerified for image verification completion
 */
OADProfile_AppCommand_e App_OADCallbackMsOad(OADProfile_App_Msg_e msg, void *pData)
{
    (void)pData; // Currently unused, reserved for future use
    OADProfile_AppCommand_e cmd = OAD_PROFILE_CANCEL;

    switch(msg)
    {
        case OAD_PROFILE_MSG_NEW_IMG_IDENTIFY:
        {
            // Validate new image version and save it if acceptable.
            // Only proceed if MS-OAD is idle and version validation passes.
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallbackMsOad: New image identify");

            if (gAppMsOadData.msOadState == MS_OAD_IDLE &&
                App_validateNewVersion((struct image_header *)pData) == true)
            {
                // Save the identified image version for MS-OAD initialization
                gAppMsOadData.identifiedImageVersion = ((struct image_header *)pData)->ih_ver;

                // Mark that image has been identified
                gAppMsOadData.oadFlags |= APP_OAD_STATE_IMAGE_IDENTIFIED;

                cmd = OAD_PROFILE_PROCEED;
            }
            else
            {
                cmd = OAD_PROFILE_CANCEL;
            }

            break;
        }
        case OAD_PROFILE_MSG_GET_MS_OAD_STATE:
        {
            // Return current MS-OAD state to be sent to the OAD client.
            // State indicates system readiness (IDLE, NEW_IMAGE_READY, etc.)
            *((uint8*)pData) = (uint8) gAppMsOadData.msOadState;
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_MS_OAD_INIT_REQ:
        {
            // Initialize MS-OAD process with the identified image version.
            // This resets the device and begins multistep OAD on next boot.
            if (gAppMsOadData.msOadState == MS_OAD_IDLE &&
                (gAppMsOadData.oadFlags & APP_OAD_STATE_IMAGE_IDENTIFIED))
            {
                // Device will reset during this call
                MSOAD_InitAndReset(&gAppMsOadData.identifiedImageVersion);

                // TODO: Handle possible error in MSOAD_InitAndReset()
                // TODO: Set the reset to happen after a short time so we can response with success?
                cmd = OAD_PROFILE_PROCEED;
            }
            else
            {
                cmd = OAD_PROFILE_CANCEL;
            }

            break;
        }
        case OAD_PROFILE_MSG_MS_OAD_COMMIT:
        {
            // Verify (commit) the new image after successful testing.
            // Only allowed when new image is ready for verification.
            if (gAppMsOadData.msOadState == MS_OAD_NEW_IMAGE_READY ||
                gAppMsOadData.msOadState == MS_OAD_IMAGE_READY_REVERT)
            {
                // Mark image as verified - device will reset
                if (MSOAD_ImageVerified() == SUCCESS)
                {
                    cmd = OAD_PROFILE_PROCEED;
                }
            }

            break;
        }
        case OAD_PROFILE_MSG_REVOKE_IMG_HDR:
        case OAD_PROFILE_MSG_START_DOWNLOAD:
        case OAD_PROFILE_MSG_FINISH_DOWNLOAD:
        case OAD_PROFILE_MSG_CANCEL_DOWNLOAD:
        case OAD_PROFILE_MSG_RESET_REQ:
        case OAD_PROFILE_MSG_DOWNLOAD_FAILED:
        case OAD_PROFILE_MSG_TIMEOUT:
        {
            // These events are handled internally by MS-OAD profile, not by application.
            // Application should not override these events in MS-OAD mode.
            cmd = OAD_PROFILE_CANCEL;
            break;
        }
        default:
        {
            // Unknown event - reject
            cmd = OAD_PROFILE_CANCEL;
            break;
        }
    }
    return (cmd);
}
#else
/**
 * @brief OAD Profile callback for standard (non-multistep) OAD
 *
 * Application callback for standard OAD operations. Handles OAD lifecycle
 * events and allows application to monitor or control OAD progress.
 *
 * @param msg OAD Profile event/message type
 * @param pData Event-specific data
 *
 * @return OAD_PROFILE_PROCEED to allow operation, OAD_PROFILE_CANCEL to reject
 *
 * @see App_OADCallbackMsOad for MS-OAD callback
 * @see OADProfile_App_Msg_e for event descriptions and which require intervention
 */
OADProfile_AppCommand_e App_OADCallback(OADProfile_App_Msg_e msg, void *pData)
{
    OADProfile_AppCommand_e cmd = OAD_PROFILE_PROCEED;

    switch(msg)
    {
        case OAD_PROFILE_MSG_REVOKE_IMG_HDR:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Revoke image header");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_NEW_IMG_IDENTIFY:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: New image identify");

            if (App_validateNewVersion((struct image_header *)pData) == true)
            {
                cmd = OAD_PROFILE_PROCEED;
            }
            else
            {
                cmd = OAD_PROFILE_CANCEL;
            }
            break;
        }
        case OAD_PROFILE_MSG_START_DOWNLOAD:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Download new image");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_FINISH_DOWNLOAD:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Download complete");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_CANCEL_DOWNLOAD:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Cancel download");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_RESET_REQ:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Reset device");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_DOWNLOAD_FAILED:
        {
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: Download failed");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
        case OAD_PROFILE_MSG_TIMEOUT:
        {
            // OAD operation timed out (e.g., no activity within timeout period)
            MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE1, 0, "App_OADCallback: OAD timeout");
            cmd = OAD_PROFILE_PROCEED;
            break;
        }
    }
    return (cmd);
}
#endif

/*********************************************************************
 * @fn      OAD_start
 *
 * @brief   Initialize and register the OAD profile and service
 *
 * Called after BLE stack initialization to set up the OAD profile.
 * Behavior differs based on OAD mode:
 *
 * - Standard OAD: Registers App_OADCallback
 * - MS-OAD: Registers App_OADCallbackMsOad and queries MS-OAD state
 *
 * For MS-OAD, retrieves the current state to determine system readiness
 * (idle, new image needing verification, etc.).
 *
 * @return  SUCCESS or stack call status
 *
 * @note For MS-OAD, MSOAD_init() must be called before this function.
 */
bStatus_t OAD_start(void)
{
  bStatus_t status = SUCCESS;

#ifdef MS_OAD
  // Register MS-OAD specific callback
  OADProfile_start(&App_OADCallbackMsOad);

  // Get current MS-OAD state (idle, new image ready, etc.)
  gAppMsOadData.msOadState = MSOAD_GetState();

  // Reset application-level OAD tracking
  gAppMsOadData.oadFlags = APP_OAD_STATE_IDLE;
  gAppMsOadData.identifiedImageVersion = (struct image_version){0,0,0,0};

#ifdef GAP_BOND_MGR
  // If new image is ready or ready for verification, send service change indication to trigger client rediscovery.
  // This is important for clients to discover the new OAD characteristics after rebooting into the new image,
  // especially if the new image has a different GAP service configurations
  if (gAppMsOadData.msOadState == MS_OAD_NEW_IMAGE_READY ||
      gAppMsOadData.msOadState == MS_OAD_IMAGE_READY_REVERT)
  {
    GAPBondMgr_ServiceChangeInd(0xFFFF, TRUE);
  }
#endif

#else
  // Register standard OAD callback
  OADProfile_start(&App_OADCallback);
#endif

  //APP_HDR_ADDR is the place in the flash memory of APP header, and it is imported from the predefined symbols
  //SwUpdate_GetSWVersion function extract image version struct from given address
  struct image_version * img_ver = (struct image_version *)SwUpdate_GetSWVersion(APP_HDR_ADDR);
  MenuModule_printf(APP_MENU_PROFILE_STATUS_LINE, 0, "App version: %d.%d.%d.%d",
                    img_ver->iv_major,
                    img_ver->iv_minor,
                    img_ver->iv_revision,
                    img_ver->iv_build_num);
  return ( status );
}

#endif // defined (OAD_CFG) && !defined(OAD_PERSISTENT)
