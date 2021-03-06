/*! *********************************************************************************
* \addtogroup BLE
* @{
********************************************************************************** */
/*!
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
*
* \file
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of Freescale Semiconductor, Inc. nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/************************************************************************************
*************************************************************************************
* DO NOT MODIFY THIS FILE!
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/
#include "ble_general.h"
#include "att_errors.h"
#include "ble_config.h"

#include "ModuleInfo.h"

/************************************************************************************
*************************************************************************************
* Public memory declarations - external references from Host library
*************************************************************************************
************************************************************************************/
uint8_t gcGapMaximumBondedDevices_d = gMaxBondedDevices_c;
bleBondIdentityHeaderBlob_t gaBondIdentityHeaderBlobs[gMaxBondedDevices_c];

uint8_t gcGattMaxHandleCountForWriteNotifications_c = gMaxWriteNotificationHandles_c;
uint16_t gGattWriteNotificationHandles[gMaxWriteNotificationHandles_c];
uint8_t gcGattMaxHandleCountForReadNotifications_c = gMaxReadNotificationHandles_c;
uint16_t gGattReadNotificationHandles[gMaxReadNotificationHandles_c];

uint8_t gcGattDbMaxPrepareWriteOperationsInQueue_c = gPrepareWriteQueueSize_c;
attPrepareWriteRequestParams_t gPrepareWriteQueues[gcGattDbMaxPrepareWriteClients_c][gPrepareWriteQueueSize_c];

uint16_t gGapDefaultTxOctets = gBleDefaultTxOctets_c;
uint16_t gGapDefaultTxTime = gBleDefaultTxTime_c;

uint16_t gGapHostPrivacyTimeout = gBleHostPrivacyTimeout_c;
uint16_t gGapControllerPrivacyTimeout = gBleControllerPrivacyTimeout_c;

bool_t gGapLeSecureConnectionsOnlyMode = gBleLeSecureConnectionsOnlyMode_c;
bool_t gGapLeScOobHasMitmProtection = gBleLeScOobHasMitmProtection_c;

/* The following definitions are required by the VERSION_TAGS. DO NOT MODIFY or REMOVE */
extern const moduleInfo_t BLE_HOST_VERSION;
#if defined ( __IAR_SYSTEMS_ICC__ )
#pragma required=BLE_HOST_VERSION /* force the linker to keep the symbol in the current compilation unit */
uint8_t ble_dummy; /* symbol suppressed by the linker as it is unused in the compilation unit, but necessary because 
                             to avoid warnings related to #pragma required */
#elif defined(__GNUC__)
static const moduleInfo_t *const dummy __attribute__((__used__)) = &BLE_HOST_VERSION;
#endif /* __IAR_SYSTEMS_ICC__ */
/*! *********************************************************************************
* @}
********************************************************************************** */
