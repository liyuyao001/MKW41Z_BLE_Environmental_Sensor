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

#ifndef _HDC1080_TASK_CONFIG_H_
#define _HDC1080_TASK_CONFIG_H_

#include "fsl_os_abstraction.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

/*
 * These values should be modified by the application as necessary.
 * They are used by the task initialization code from ble_host_tasks.c.
 */

#ifndef gHDC1080_TaskStackSize_c
#define gHDC1080_TaskStackSize_c 1300
#endif

#ifndef gHDC1080_TaskPriority_c
#define gHDC1080_TaskPriority_c 10
#endif

//*****************************************************************************
// #defines
//*****************************************************************************
#define HDC1080_ADDRESS 		0x40	//I2C address for HDC1080
#define TEMPERATURE_ADDRESS 	0x00	//Temperature measurement output register
#define HUMIDITY_ADDRESS		0x01	//Relative humidity meas. output register
#define CONFIGURATION_ADDRESS 	0x02	//HDC1080 configuration and status
#define TEMP_RH_11BIT_MSB		0x15	//MSB of configuration (sets Temp & RH measurements, 11-bit resolution)
#define TEMP_RH_11BIT_LSB		0x00	//LSB of configuration (sets Temp & RH measurements, 11-bit resolution)
#define SERIAL_ID_HIGH			0xFB
#define SERIAL_ID_MID			0xFC
#define SERIAL_ID_LOW			0xFD

//#define HDC1080_CONFIG_RST    (1 << 15)
//#define HDC1080_CONFIG_HEAT   (1 << 13)
//#define HDC1080_CONFIG_MODE   (1 << 12)
//#define HDC1080_CONFIG_BATT   (1 << 11)
//#define HDC1080_CONFIG_TRES_14  0
//#define HDC1080_CONFIG_TRES_11  (1 << 10)
//#define HDC1080_CONFIG_HRES_14  0
//#define HDC1080_CONFIG_HRES_11  (1 << 8)
//#define HDC1080_CONFIG_HRES_8   (1 << 9)
//
//#define NEWCONFIG   (HDC1080_CONFIG_RST | HDC1080_CONFIG_HEAT | HDC1080_CONFIG_MODE | HDC1080_CONFIG_TRES_14 | HDC1080_CONFIG_HRES_14)

////*****************************************************************************
//// Global Variables
////*****************************************************************************
//extern uint8_t 	i2c_transmitCounter;	//Variable to store transmit status for I2C
//extern uint8_t 	*p_i2c_transmitData;	//Pointer to I2C transmit data
//extern uint8_t 	*p_i2c_receivedData;	//Pointer to I2C received data
extern int32_t		HDC_Temp;			//Variable to store temperature
extern uint32_t 	HDC_RH;				//Variable to store humidity

//*****************************************************************************
// Function declarations
//*****************************************************************************
void HDC1080_init(void);
void HDC1080_startMeasurement(void);
void HDC1080_readMeasurement(void);

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/*! *********************************************************************************
* \brief  Initializes the two tasks of the HDC1080 Stack.
*
* \return  osaStatus_t.
*
********************************************************************************** */
osaStatus_t HDC1080_TaskInit(void);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_HOST_TASK_CONFIG_H_ */

/*! *********************************************************************************
* @}
********************************************************************************** */
