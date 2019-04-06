/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v1.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "stdbool.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

USBD_CDC_LineCodingTypeDef LineCoding =
{
		115200, /* baud rate*/
		0x00,   /* stop bits-1*/
		0x00,   /* parity - none*/
		0x08    /* nb. of bits 8*/
};

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  10
#define APP_TX_DATA_SIZE  255
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
//uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

#define RXBUFFERSIZE	4
char usb_commandlinebuffer[RXBUFFERSIZE];
uint16_t usb_commandlinebuffer_pos = 0;
uint8_t UserRxBufferFS[RXBUFFERSIZE];
uint8_t rxbuffer[RXBUFFERSIZE];
uint8_t auxRXbuffer[RXBUFFERSIZE];
uint8_t LenData=0;
uint32_t usb_bytes_received = 0;
uint32_t usb_bytes_scanned = 0;
uint32_t last_string_start_pos = 0;
uint8_t usb_rxbuffer_overflow = 0;

uint32_t bytes_sent = 0;
uint32_t bytes_written = 0;
bool foundNewLine = false;

// variáveis aauxiliares para obter o valor de setpoint recebido pela porta COM
uint8_t received_data [4];
uint32_t received_data_size;
uint32_t receive_total = 0;

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
	/* Set Application Buffers */
	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
	return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
	return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
	switch(cmd)
	{
	case CDC_SEND_ENCAPSULATED_COMMAND:

		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:

		break;

	case CDC_SET_COMM_FEATURE:

		break;

	case CDC_GET_COMM_FEATURE:

		break;

	case CDC_CLEAR_COMM_FEATURE:

		break;

		/*******************************************************************************/
		/* Line Coding Structure                                                       */
		/*-----------------------------------------------------------------------------*/
		/* Offset | Field       | Size | Value  | Description                          */
		/* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
		/* 4      | bCharFormat |   1  | Number | Stop bits                            */
		/*                                        0 - 1 Stop bit                       */
		/*                                        1 - 1.5 Stop bits                    */
		/*                                        2 - 2 Stop bits                      */
		/* 5      | bParityType |  1   | Number | Parity                               */
		/*                                        0 - None                             */
		/*                                        1 - Odd                              */
		/*                                        2 - Even                             */
		/*                                        3 - Mark                             */
		/*                                        4 - Space                            */
		/* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
		/*******************************************************************************/
	case CDC_SET_LINE_CODING:
		LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
		LineCoding.format     = pbuf[4];
		LineCoding.paritytype = pbuf[5];
		LineCoding.datatype   = pbuf[6];
		break;

	case CDC_GET_LINE_CODING:
		pbuf[0] = (uint8_t)(LineCoding.bitrate);
		pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
		pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
		pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
		pbuf[4] = LineCoding.format;
		pbuf[5] = LineCoding.paritytype;
		pbuf[6] = LineCoding.datatype;
		break;

	case CDC_SET_CONTROL_LINE_STATE:

		break;

	case CDC_SEND_BREAK:

		break;

	default:
		break;
	}

	return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
//static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
//{
//  /* USER CODE BEGIN 6 */
////	USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
////	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
////	return (USBD_OK);
//  /* USER CODE END 6 */
//}

static int8_t CDC_Receive_FS (uint8_t* Buf, uint32_t *Len)
{
    uint32_t len1 = *Len;
    uint16_t current_pos;

    /*
     * bytes_received, last_string_start_pos, bytes_scanned are all absolute number and
     * wrapped into the ring buffer using the modulo operation like bytes_received % RXBUFFERSIZE
     *
     * The last_string_start_pos is the position of the last \n character, indicating a new command line
     *
     *                              received
                                       |
     *                                 |<--len1-->|
     *    +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++(rxbuffer)
     *      |                                |                                |
     *      | <--------RXBUFFERSIZE--------> | <--------RXBUFFERSIZE--------> | <-...
     *    last_string
     */
    if (usb_bytes_received + len1 > last_string_start_pos + RXBUFFERSIZE)
    {
        /*
         * The received packet does not fit into the remain space of the ring buffer.
         * That means data will be lost. But at least try to fit as much of the data into the ring buffer
         * as possible. Could be a very large packet being received at once and a \n is in the middle?
         *
         * Hence test if there is space available at all. If not, this is an overflow, else reduce
         * the len to what will fit int the remaining space.
         */
        if (usb_bytes_received > last_string_start_pos + RXBUFFERSIZE)
        {
            // overflow condition
            usb_rxbuffer_overflow = 1;
        }
        else
        {
            len1 = last_string_start_pos + RXBUFFERSIZE - usb_bytes_received;
        }
    }

    if (usb_rxbuffer_overflow == 0)
    {
        /*
         * Copy the USB packet into the ring buffer.
         * There are two cases, either the ring buffer runs over the end and the received packet
         * has to be split into a part stored at the end of the rxbuffer ring buffer and the beginning.
         *
         * Or the packet fits into the middle of the ring buffer, hence is just copied there.
         */
        current_pos = usb_bytes_received % RXBUFFERSIZE;
        if (len1 > ((uint16_t) (RXBUFFERSIZE - current_pos)))
        {
            memcpy(&rxbuffer[current_pos], Buf, RXBUFFERSIZE - current_pos);
            memcpy(&rxbuffer[0], &Buf[RXBUFFERSIZE - current_pos], len1 + current_pos - RXBUFFERSIZE);
            memcpy(&received_data[current_pos], Buf, RXBUFFERSIZE - current_pos);
        }
        else
        {
            memcpy(&rxbuffer[current_pos], Buf, len1);
            memcpy(&received_data[current_pos], Buf, len1);
        }
        usb_bytes_received += len1;
    }

    /* Prepare for the next reception of data */
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    return (USBD_OK);
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
//	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
//	if (hcdc == 0){
//		return USBD_FAIL;
//	}
//
//	if (hcdc->TxState != 0){
//		return USBD_BUSY;
//	}
//
//	memcpy(UserTxBufferFS, Buf, sizeof(char) * Len);
//	USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
//
//	result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
//	while(hcdc->TxState);
//
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

void USB_Transmit()
{
	uint32_t buffptr = bytes_sent % APP_TX_DATA_SIZE;
	uint32_t buffsize = bytes_written - bytes_sent;

	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
	{
		if(bytes_written != bytes_sent)
		{
			if (buffptr + buffsize > APP_TX_DATA_SIZE)
			{
				buffsize = APP_TX_DATA_SIZE - buffptr;
			}

			USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t*)&UserTxBufferFS[buffptr], buffsize);

			if(USBD_CDC_TransmitPacket(&hUsbDeviceFS) == USBD_OK)
			{
				bytes_sent += buffsize;
			}
		}
	}
}

uint8_t USB_TransmitBuffer(uint8_t *ptr, uint32_t len)
{
	uint32_t le;
	uint32_t rel_pos = bytes_written % APP_TX_DATA_SIZE;

	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && len < APP_RX_DATA_SIZE)
	{
		if (rel_pos + len > APP_TX_DATA_SIZE)
		{
			le = APP_TX_DATA_SIZE - rel_pos;
			memcpy(&UserTxBufferFS[rel_pos], ptr, le);
			memcpy(UserTxBufferFS, &ptr[le], len - le);
		}
		else
		{
			memcpy(&UserTxBufferFS[rel_pos], ptr, len);
		}
		bytes_written += len;

		// Synchronous transmission
		// Call outside of this function for asynchronous transmission (comment line)
		USB_Transmit();
	}

	return USBD_OK;
}


bool USB_CanReadLine()
{
	uint32_t idx = usb_bytes_scanned;

    while (idx < usb_bytes_received)
    {
        uint8_t c = rxbuffer[usb_bytes_scanned % RXBUFFERSIZE];
        idx++;

        if (c == '\n' || c == '\r')
        {
        	return true;
        }
    }

	return false;
}

/** \brief USB_ReceiveString
 *         Does extract an entire line from the USB buffer using the line terminator
 *         character \r and/or \n. If there was an overflow, the line is ignored.
 *
 * This method is called periodically and tries to catchup with the received USB packets
 * by scanning from the last scanned position up to the last received character.
 *
 * bytes_scanned is the absolute position of the last byte read
 * bytes_received is the absolute position of the last received byte
 *
 * The ring buffer rxbuffer is scanned for \n chars and everything between to \n chars is
 * copied into a line buffer.
 * Hence the line buffer starts with the first char after a \n and ends
 *
 * \return uint16_t returns 1 in case a line was found
 *
 */
uint16_t USB_ReceiveString()
{
    /*
    * Go through all characters and locate the next \n. If one is found
    * copy the entire text from the start position to the position of the next found \n
    * character into the commandlinebuffer.
    */
    while (usb_bytes_scanned < usb_bytes_received)
    {
        uint8_t c = rxbuffer[usb_bytes_scanned % RXBUFFERSIZE];
        usb_bytes_scanned++;
        //CDC_TransmitBuffer((uint8_t *) &c, 1); // echo input text
        if (c == '\n' || c == '\r')
        {
            last_string_start_pos = usb_bytes_scanned; // next string starts here
            if (usb_commandlinebuffer_pos < RXBUFFERSIZE-1u)   // in case the string does not fit into the commandlinebuffer, the entire line is ignored
            {
                usb_commandlinebuffer[usb_commandlinebuffer_pos++] = c;
                usb_commandlinebuffer_pos = 0;
                return 1;
            }
            usb_commandlinebuffer_pos = 0;
            //received_data=(uint32_t)atoi((char*)usb_commandlinebuffer);
            memcpy(received_data, usb_commandlinebuffer, usb_bytes_received);  // copia para a variável receivel_data, o valor recebido na porta COM
        }
        else if (c == 0x08) // backspace char
        {
            /*
             *                      commandlinebuffer_pos
             *                               |
             * ....... 0x65 0x66 0x67 0x68 0x08
             * User deleted char 0x68, hence instead of moving the commandlinebuffer_pos one ahead, it is
             * moved backwards by one step.
             */
            if (usb_commandlinebuffer_pos > 0)
            {
                usb_commandlinebuffer_pos--;
            }
        }
        else
        {
            if (usb_commandlinebuffer_pos < RXBUFFERSIZE)
            {
                usb_commandlinebuffer[usb_commandlinebuffer_pos++] = c;
            }
        }
    }
    // No newline char was found
    if (usb_rxbuffer_overflow == 1)
    {
        usb_bytes_scanned = usb_bytes_received;
        last_string_start_pos = usb_bytes_received;
        usb_rxbuffer_overflow = 0;
    }
    return 0;
}

uint16_t USB_ReceiveData()
{

	return rxbuffer;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
