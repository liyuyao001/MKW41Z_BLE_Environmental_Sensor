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
* Include
*************************************************************************************
************************************************************************************/
#include "Messaging.h"
#include "HDC1080_task_config.h"
#include "fsl_os_abstraction.h"
#include "Panic.h"

/*  SDK Included Files */
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_gpio.h"
#include "fsl_port.h"



/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/
osaTaskId_t  gHDC1080_TaskId;
osaEventId_t gHDC1080_TaskEvent;

//msgQueue_t   gApp2Host_TaskQueue;
//msgQueue_t   gHci2Host_TaskQueue;

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define I2C_READ_FROM_HDC1080

#define ACCEL_I2C_CLK_SRC I2C1_CLK_SRC
#define ACCEL_I2C_CLK_FREQ CLOCK_GetFreq(I2C1_CLK_SRC)

#define I2C_RELEASE_SDA_PORT PORTC
#define I2C_RELEASE_SCL_PORT PORTC
#define I2C_RELEASE_SDA_GPIO GPIOC
#define I2C_RELEASE_SDA_PIN 3U
#define I2C_RELEASE_SCL_GPIO GPIOC
#define I2C_RELEASE_SCL_PIN 2U
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 100000U
#define FXOS8700_WHOAMI 0xC7U
#define MMA8451_WHOAMI 0x1AU
#define ACCEL_STATUS 0x00U
#define ACCEL_XYZ_DATA_CFG 0x0EU
#define ACCEL_CTRL_REG1 0x2AU
/* FXOS8700 and MMA8451 have the same who_am_i register address. */
#define ACCEL_WHOAMI_REG 0x0DU
#define ACCEL_READ_TIMES 10U

#define HDC_I2C_CLK_SRC I2C1_CLK_SRC
#define HDC_I2C_CLK_FREQ CLOCK_GetFreq(I2C1_CLK_SRC)

#define HDC1080_WHOAMI 0x5010U			//0x1050
#define HDC_WHOAMI_REG 0xFFU


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);

static bool I2C_ReadAccelWhoAmI(void);
static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);

static bool I2C_ReadHDCWhoAmI(void);
static bool I2C_WriteHDCReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadHDCRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*  FXOS8700 and MMA8451 device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};
/*  HDC1080 device address */
const uint8_t g_HDC_address[] = {0x40U};
i2c_master_handle_t g_m_handle;

uint8_t 	HDC_Data[4] = {0};					//Buffer for received bytes
//*****************************************************************************
// Global Variables
//*****************************************************************************
uint8_t 	i2c_transmitCounter;	//Variable to store transmit status for I2C
uint8_t 	*p_i2c_transmitData;	//Pointer to I2C transmit data
uint8_t 	*p_i2c_receivedData;	//Pointer to I2C received data
int32_t		HDC_Temp;			//Variable to store temperature
uint32_t 	HDC_RH;				//Variable to store humidity

uint8_t g_accel_addr_found = 0x00;

volatile bool completionFlag = false;
volatile bool nakFlag = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA low */
    for (i = 0; i < 9; i++)
    {
        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
    {
        nakFlag = true;
    }
}

static bool I2C_ReadAccelWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */
    uint8_t who_am_i_reg = ACCEL_WHOAMI_REG;
    uint8_t who_am_i_value = 0x00;
    uint8_t accel_addr_array_size = 0x00;
    bool find_device = false;
    uint8_t i = 0;
    uint32_t sourceClock = 0;

    i2c_master_config_t masterConfig;

    /*
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.enableStopHold = false;
     * masterConfig.glitchFilterWidth = 0U;
     * masterConfig.enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    sourceClock = ACCEL_I2C_CLK_FREQ;

    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &masterConfig, sourceClock);

    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = g_accel_address[0];
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &who_am_i_reg;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferNoStopFlag;

    accel_addr_array_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);

    for (i = 0; i < accel_addr_array_size; i++)
    {
        masterXfer.slaveAddress = g_accel_address[i];

        I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            find_device = true;
            g_accel_addr_found = masterXfer.slaveAddress;
            break;
        }
    }

    if (find_device == true)
    {
        masterXfer.direction = kI2C_Read;
        masterXfer.subaddress = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data = &who_am_i_value;
        masterXfer.dataSize = 1;
        masterXfer.flags = kI2C_TransferRepeatedStartFlag;

        I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            if (who_am_i_value == FXOS8700_WHOAMI)
            {
                PRINTF("Found an FXOS8700 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else if (who_am_i_value == MMA8451_WHOAMI)
            {
                PRINTF("Found an MMA8451 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else
            {
                PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
                PRINTF("It's not MMA8451 or FXOS8700. \r\n");
                PRINTF("The device address is 0x%x. \r\n", masterXfer.slaveAddress);
                return false;
            }
        }
        else
        {
            PRINTF("Not a successful i2c communication \r\n");
            return false;
        }
    }
    else
    {
        PRINTF("\r\n Do not find an accelerometer device ! \r\n");
        return false;
    }
}

static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

void I2C_ACCEL_TaskHandler()
{
	while(1)
	{
		PRINTF("\r\nI2C example -- Read Accelerometer Value\r\n");
	    bool isThereAccel = false;

		    I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
		    isThereAccel = I2C_ReadAccelWhoAmI();

		    /*  read the accel xyz value if there is accel device on board */
		    if (true == isThereAccel)
		    {
		        uint8_t databyte = 0;
		        uint8_t write_reg = 0;
		        uint8_t readBuff[7];
		        int16_t x, y, z;
		        uint8_t status0_value = 0;
		        uint32_t i = 0U;

		        /*  please refer to the "example FXOS8700CQ Driver Code" in FXOS8700 datasheet. */
		        /*  write 0000 0000 = 0x00 to accelerometer control register 1 */
		        /*  standby */
		        /*  [7-1] = 0000 000 */
		        /*  [0]: active=0 */
		        write_reg = ACCEL_CTRL_REG1;
		        databyte = 0;
		        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

		        /*  write 0000 0001= 0x01 to XYZ_DATA_CFG register */
		        /*  [7]: reserved */
		        /*  [6]: reserved */
		        /*  [5]: reserved */
		        /*  [4]: hpf_out=0 */
		        /*  [3]: reserved */
		        /*  [2]: reserved */
		        /*  [1-0]: fs=01 for accelerometer range of +/-4g range with 0.488mg/LSB */
		        /*  databyte = 0x01; */
		        write_reg = ACCEL_XYZ_DATA_CFG;
		        databyte = 0x01;
		        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);

		        /*  write 0000 1101 = 0x0D to accelerometer control register 1 */
		        /*  [7-6]: aslp_rate=00 */
		        /*  [5-3]: dr=001 for 200Hz data rate (when in hybrid mode) */
		        /*  [2]: lnoise=1 for low noise mode */
		        /*  [1]: f_read=0 for normal 16 bit reads */
		        /*  [0]: active=1 to take the part out of standby and enable sampling */
		        /*   databyte = 0x0D; */
		        write_reg = ACCEL_CTRL_REG1;
		        databyte = 0x0d;
		        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);
		        PRINTF("The accel values:\r\n");
		        for (i = 0; i < ACCEL_READ_TIMES; i++)
		        {
		            status0_value = 0;
		            /*  wait for new data are ready. */
		            while (status0_value != 0xff)
		            {
		                I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, ACCEL_STATUS, &status0_value, 1);
		            }

		            /*  Multiple-byte Read from STATUS (0x00) register */
		            I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, ACCEL_STATUS, readBuff, 7);

		            status0_value = readBuff[0];
		            x = ((int16_t)(((readBuff[1] * 256U) | readBuff[2]))) / 4U;
		            y = ((int16_t)(((readBuff[3] * 256U) | readBuff[4]))) / 4U;
		            z = ((int16_t)(((readBuff[5] * 256U) | readBuff[6]))) / 4U;

		            PRINTF("status_reg = 0x%x , x = %5d , y = %5d , z = %5d \r\n", status0_value, x, y, z);
		        }
		    }

		    PRINTF("\r\nEnd of I2C example .\r\n");

		    OSA_TimeDelay(200);
	}
}

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/
static void HDC1080_Task(osaTaskParam_t argument);
/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/
OSA_TASK_DEFINE(HDC1080_Task, gHDC1080_TaskPriority_c, 1, gHDC1080_TaskStackSize_c, FALSE);

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

osaStatus_t HDC1080_TaskInit(void)
{     
    /* Already initialized? */
    if(gHDC1080_TaskId)
    {      
      return osaStatus_Error;
    }
    
    /* Initialization of task related */
    gHDC1080_TaskEvent = OSA_EventCreate(TRUE);
    if( gHDC1080_TaskEvent == NULL)
    {
        return osaStatus_Error;
    }

    BOARD_InitPins();
    BOARD_BootClockRUN();
//    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    BOARD_InitDebugConsole();


    /* Task creation */
     
    gHDC1080_TaskId = OSA_TaskCreate(OSA_TASK(HDC1080_Task), NULL);
    
    if( NULL == gHDC1080_TaskId )
    {
        panic(0,0,0,0);
        return osaStatus_Error;
    }

    return osaStatus_Success;
}

/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

static void HDC1080_Task(osaTaskParam_t argument)
{
    HDC1080_TaskHandler((void *) NULL);
//	I2C_ACCEL_TaskHandler((void *) NULL);
}

/*! *********************************************************************************
* @}
********************************************************************************** */
void HDC1080_TaskHandler()
{
	PRINTF("\r\nI2C example -- Read Humidity and Temperature Sensor Value\r\n");
    bool isThereHDC1080 = false;

	I2C_MasterTransferCreateHandle(BOARD_HDC_I2C_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
	isThereHDC1080 = I2C_ReadHDCWhoAmI();

	/*  read the accel xyz value if there is accel device on board */
	if (true == isThereHDC1080)
	{
		HDC1080_init();
//		    	HDC1080_startMeasurement();

	}

	PRINTF("\r\nHDC1080 Initialized .\r\n");


	while(1)
	{
		OSA_TimeDelay(10);
//		I2C_ReadHDCWhoAmI();
		HDC1080_startMeasurement();
		OSA_TimeDelay(500);
		HDC1080_readMeasurement();
//		    vTaskSuspend(NULL);

	}
}


static bool I2C_ReadHDCWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */
    uint8_t who_am_i_reg = HDC_WHOAMI_REG;
    uint16_t who_am_i_value = 0x00;
    uint8_t HDC_addr_array_size = 0x00;
    bool find_device = false;
    uint8_t i = 0;
    uint32_t sourceClock = 0;

    i2c_master_config_t masterConfig;

    /*
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.enableStopHold = false;
     * masterConfig.glitchFilterWidth = 0U;
     * masterConfig.enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps = I2C_BAUDRATE;

    sourceClock = HDC_I2C_CLK_FREQ;

    I2C_MasterInit(BOARD_HDC_I2C_BASEADDR, &masterConfig, sourceClock);

    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = g_HDC_address[0];
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &who_am_i_reg;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferNoStopFlag;

    HDC_addr_array_size = sizeof(g_HDC_address) / sizeof(g_HDC_address[0]);

    for (i = 0; i < HDC_addr_array_size; i++)
    {
        masterXfer.slaveAddress = g_HDC_address[i];

        I2C_MasterTransferNonBlocking(BOARD_HDC_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            find_device = true;
            g_accel_addr_found = masterXfer.slaveAddress;
            break;
        }
    }

    if (find_device == true)
    {
        masterXfer.direction = kI2C_Read;
        masterXfer.subaddress = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data = &who_am_i_value;
        masterXfer.dataSize = 2;
        masterXfer.flags = kI2C_TransferRepeatedStartFlag;

        I2C_MasterTransferNonBlocking(BOARD_HDC_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            if (who_am_i_value == HDC1080_WHOAMI)
            {
                PRINTF("Found an HDC1080 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else
            {
                PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
                PRINTF("It's not HDC1080. \r\n");
                PRINTF("The device address is 0x%x. \r\n", masterXfer.slaveAddress);
                return false;
            }
        }
        else
        {
            PRINTF("Not a successful i2c communication \r\n");
            return false;
        }
    }
    else
    {
        PRINTF("\r\n Do not find an accelerometer device ! \r\n");
        return false;
    }
}

static bool I2C_WriteHDCReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

static bool I2C_ReadHDCRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }
}

void HDC1080_init(void)
{
	i2c_master_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));

	static const uint8_t HDC1080_CONFIGURATION_FRAME[3] =
	{
		//Configuration register address
		CONFIGURATION_ADDRESS,
		//MSB of configuration (sets Temp & Humidity measurements, 11-bit resolution)
		TEMP_RH_11BIT_MSB,
		//LSB of configuration
		TEMP_RH_11BIT_LSB
	};

		// Set TX pointer to HDC1080_CONFIGURATION_FRAME
    	p_i2c_transmitData = (uint8_t *)HDC1080_CONFIGURATION_FRAME;

    	//Load transmit byte counter
    	i2c_transmitCounter = sizeof HDC1080_CONFIGURATION_FRAME;

//		masterXfer.slaveAddress = HDC1080_ADDRESS;
//		masterXfer.direction = kI2C_Write;
//		masterXfer.subaddress = CONFIGURATION_ADDRESS;
//		masterXfer.subaddressSize = 1;
//		masterXfer.data = &HDC1080_CONFIGURATION_FRAME[1];
//		masterXfer.dataSize = i2c_transmitCounter - 1;
//		masterXfer.flags = kI2C_TransferDefaultFlag;
////		masterXfer.flags = kI2C_TransferNoStopFlag;

	    masterXfer.slaveAddress = HDC1080_ADDRESS;
	    masterXfer.direction = kI2C_Write;
	    masterXfer.subaddress = 0;
	    masterXfer.subaddressSize = 0;
	    masterXfer.data = p_i2c_transmitData;
	    masterXfer.dataSize = i2c_transmitCounter;
	    masterXfer.flags = kI2C_TransferDefaultFlag;

	    /*  direction=write : start+device_write;cmdbuff;xBuff; */
	    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

	    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

	    /*  wait for transfer completed. */
	    while ((!nakFlag) && (!completionFlag))
	    {
	    }

	    nakFlag = false;

	    if (completionFlag == true)
	    {
	        completionFlag = false;
	        return true;
	    }
	    else
	    {
	        return false;
	    }

}

//*****************************************************************************
//
// Send sensor a message to sample temperature and humidity.
//
//
// This function sends the HDC1080 the command to start measurement. HDC1080 will
// interrupt MSP (IO) when data is available for a read.
//
// return None.
//
//*****************************************************************************
void HDC1080_startMeasurement(void)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));


	static const uint8_t HDC1080_START_MEAS_FRAME[1] =
	{
		TEMPERATURE_ADDRESS 	//Temp. register address
//		SERIAL_ID_HIGH
	};

	//Transmit array start address
	p_i2c_transmitData = (uint8_t *)HDC1080_START_MEAS_FRAME;
	//Load transmit byte counter
	i2c_transmitCounter = sizeof HDC1080_START_MEAS_FRAME;

    masterXfer.slaveAddress = HDC1080_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = p_i2c_transmitData;
    masterXfer.dataSize = i2c_transmitCounter;
    masterXfer.flags = kI2C_TransferDefaultFlag;

//	masterXfer.slaveAddress = HDC1080_ADDRESS;
//	masterXfer.direction = kI2C_Write;
//	masterXfer.subaddress = 0;
//	masterXfer.subaddressSize = 0;
//	masterXfer.data = p_i2c_transmitData;
//	masterXfer.dataSize = i2c_transmitCounter;
//	masterXfer.flags = kI2C_TransferNoStopFlag;


    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

    /*  wait for transfer completed. */
    while ((!nakFlag) && (!completionFlag))
    {
    }

    nakFlag = false;

    if (completionFlag == true)
    {
        completionFlag = false;
        return true;
    }
    else
    {
        return false;
    }

}

//*****************************************************************************
//
// Read temperature and humidity from sensor
//
//
// This function reads the data from the HDC1080. HDC1080_readMeasurement is called
// after the HDC1080 triggers a ready interrupt.
//
// return None.
//
//*****************************************************************************
void HDC1080_readMeasurement(void)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    static const uint8_t HDC1080_READ_MEAS_FRAME[1] =
	{
		TEMPERATURE_ADDRESS 	//Temp. register address
//		0xFE
	};
	//Transmit array start address
	p_i2c_transmitData = (uint8_t *)HDC1080_READ_MEAS_FRAME;
	//Load transmit byte counter
	i2c_transmitCounter = sizeof HDC1080_READ_MEAS_FRAME;

	masterXfer.slaveAddress = HDC1080_ADDRESS;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = p_i2c_transmitData;
	masterXfer.dataSize = i2c_transmitCounter;
	masterXfer.flags = kI2C_TransferNoStopFlag;
	I2C_MasterTransferNonBlocking(BOARD_HDC_I2C_BASEADDR, &g_m_handle, &masterXfer);

	/*  wait for transfer completed. */
	while ((!nakFlag) && (!completionFlag))
	{
	}

	nakFlag = false;

	if (completionFlag == true)
	{
		completionFlag = false;
	}

	OSA_TimeDelay(500);

	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = 0;
	masterXfer.subaddressSize = 0;
	masterXfer.data = (uint8_t *)HDC_Data;
//	masterXfer.dataSize = 2;
	masterXfer.dataSize = sizeof HDC_Data;
	masterXfer.flags = kI2C_TransferRepeatedStartFlag;

	I2C_MasterTransferNonBlocking(BOARD_HDC_I2C_BASEADDR, &g_m_handle, &masterXfer);

	/*  wait for transfer completed. */
	while ((!nakFlag) && (!completionFlag))
	{
	}

	nakFlag = false;

	if (completionFlag == true)
	{
		completionFlag = false;
	}


	//Combine HDC Temp data bytes into one variable (allows for IQMath)
	HDC_Temp = (
			(0x00000000) |
			((uint32_t)HDC_Data[0] << 8) |
			((uint32_t)HDC_Data[1] << 0)
			);
	//Calculate temp according to HDC1080 datasheet
	HDC_Temp = ((HDC_Temp*165)>>16)-40;

	//Combine HDC RH data bytes into one 32-bit variable (allows for IQMath)
	HDC_RH = (
			(0x00000000) |
			((uint32_t)HDC_Data[2] << 8) |
			((uint32_t)HDC_Data[3] << 0)
			);

	//Calculate RH according to HDC1080 datasheet (leaves result in RH%)
	HDC_RH = (HDC_RH*100)>>16;
	PRINTF("The Temperature is %d\t", HDC_Temp);
	PRINTF("The Humidity is %d\r\n", HDC_RH);
}
