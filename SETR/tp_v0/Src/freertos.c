/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "tim.h"
#include "usb_device.h"

#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"
#include "usbd_cdc_if.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "../../BSP/STM32F411E-Discovery/stm32f411e_discovery_accelerometer.h"  //faz include das lib do acelerometro
#include <PID_v1.h>

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int16_t adc_value;
int8_t proporcional=0;
int8_t integral= 0;
int8_t derivativo=0;
int8_t kp,ki,kd;
int8_t last_erro=0;


double Setpoint, Input, Output;
PidType* myPID;
FloatType myPIDInput; // * Pointers to the Input, Output, and Setpoint variables
FloatType myPIDOutput; //   This creates a hard link between the variables and the
FloatType myPIDSetpoint; //   PID, freeing the user from having to constantly tell us
FloatType Kp=5, Ki=2, Kd=3;

int16_t samplesACC_X[10];
int16_t samplesACC_Y[10];
int16_t samplesACC_Z[10];

int32_t sumX=0;
int32_t sumY=0;
int32_t sumZ=0;

uint16_t ValueACC[3]={0,0,0};
int8_t ValueLM35=0;
int8_t setpoint=0;

bool getValueADC=false;

enum
  {
  	PERIPHERAL_USART,	// UART serial device
  	PERIPHERAL_USB 		// USB serial device
  };

void peripheralPrintf(uint16_t peripheral, char *format, ...)
 {
 	char buffer[64];

 	va_list args;
 	va_start(args, format);
 	vsprintf(buffer, format, args);
 	va_end(args);

 	switch(peripheral)
 	{
 	case PERIPHERAL_USART:
 		// TODO:
 		break;
 	case PERIPHERAL_USB:
 		USB_TransmitBuffer((uint8_t *)buffer, strlen(buffer));
 		break;
 	default:
 		break;
 	}
 }

void PID(int8_t PIDinput, int8_t PIDsetpoint);
void STD();
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


osThreadId AccReadHandle;
osThreadId TempReadHandle;
osThreadId ReceiveCOMHandle;
osThreadId testeHandle;
osThreadId CalculateSTD;

osMessageQId myQueue01Handle;
osMessageQId myQueue02Handle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartAccRead(void const * argument);
void StartTempRead(void const * argument);
void StartPID(void const * argument);
void SetPoint(void const * argument);
void CalcSTD(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* definition and creation of accRead */
  osThreadDef(AccRead, StartAccRead,  osPriorityAboveNormal, 0, 128);
  AccReadHandle = osThreadCreate(osThread(AccRead), NULL);

  osThreadDef(TempRead, StartTempRead,  osPriorityAboveNormal, 0, 128);
  TempReadHandle = osThreadCreate(osThread(TempRead), NULL);

  osThreadDef(ReceiveCOM, StartPID, osPriorityNormal, 0, 128);
  ReceiveCOMHandle = osThreadCreate(osThread(ReceiveCOM), NULL);

  osThreadDef(StdOK, CalcSTD, osPriorityAboveNormal, 0, 128);  //
  CalculateSTD = osThreadCreate(osThread(StdOK), NULL);

  osThreadDef(SetpointOK, SetPoint, osPriorityHigh, 0, 128);  //
  testeHandle = osThreadCreate(osThread(SetpointOK), NULL);


  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
  osMessageQDef(myQueue01, 16, uint16_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  osMessageQDef(myQueue02, 16, uint16_t);
  myQueue02Handle = osMessageCreate(osMessageQ(myQueue02), NULL);
  /* USER CODE BEGIN RTOS_QUEUES */

  /* add queues, ... */


  PID_init(myPID,Kp,Ki,Kd, PID_Direction_Direct);
  PID_SetMode(myPID, PID_Mode_Automatic);

  HAL_ADC_Start(&hadc1);    //start ADC

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  //inicia PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  //inicia PWM


  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */

osEvent evt;
osEvent evt2;
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  //LER VARIAVEL EXTERNA RXBUFFER


  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  Function implementing the PID function
  * @param  argument: Not used
  * @retval None
*/
void StartPID(void const * argument)
{
	for(;;)
	  {
		//peripheralPrintf(PERIPHERAL_USB, " STARTPID");
		  evt= osMessageGet(myQueue01Handle,100);

	    if(evt.status==osEventMessage)
		{
	    	uint16_t val= (uint16_t*)evt.value.p;
			if(val == 1000)
			{
				PID(ValueLM35, setpoint);
				//peripheralPrintf(PERIPHERAL_USB, "startPID");
				//peripheralPrintf(PERIPHERAL_USB, "SETPOINT:\t");
				//peripheralPrintf(PERIPHERAL_USB, "%d",setpoint);
			}
		}
	  }
}


/**
  * @brief  Function implementing the STD CALCULATE
  * @param  argument: Not used
  * @retval None
*/
void CalcSTD(void const * argument)
{
	for(;;)
	  {
		//peripheralPrintf(PERIPHERAL_USB, " STARTPID");
		evt2= osMessageGet(myQueue01Handle,100);

	    if(evt2.status==osEventMessage)
		{
	    	uint16_t val= (uint16_t*)evt2.value.p;
			if(val == 2000)
			{
				STD();
				//peripheralPrintf(PERIPHERAL_USB, " START");
			}
		}
	  }
}

/**
  * @brief  Function implementing the AccRead thread.
  * @param  argument: Not used
  * @retval None
*/
void StartAccRead(void const * argument)
{
	  BSP_ACCELERO_Init();
	  int16_t data[3]={0,0,0};

	  sumX=0;
	  sumY=0;
	  sumZ=0;

	  for(;;)
	  {

		  peripheralPrintf(PERIPHERAL_USB, "entrou na thread");
		  osDelay(300);
		  for(int i=0; i<=9; i++)
		  	{
			  BSP_ACCELERO_GetXYZ(data);
	  		 // adquire 10 amostra do ACC
		  		samplesACC_X[i]=data[0];
		  		samplesACC_Y[i]=data[1];
		  		samplesACC_Z[i]=data[2];

	  		// somatorio das amostras recolhidas
		  		sumX=sumX+ samplesACC_X[i];
		  		sumY=sumY+ samplesACC_Y[i];
		  		sumZ=sumZ+ samplesACC_Z[i];

		  		if(i==9)
		  		{
		  			osDelay(1);
		  			osMessagePut(myQueue01Handle, 2000,100);
		  			//STD();
		  			//peripheralPrintf(PERIPHERAL_USB, "A:\t");
		  		}
		  	}
//
//		peripheralPrintf(PERIPHERAL_USB, "X:\t");
//		peripheralPrintf(PERIPHERAL_USB, "%d",data[0]);
//		peripheralPrintf(PERIPHERAL_USB, " Y:\t");
//		peripheralPrintf(PERIPHERAL_USB, "%d",data[1]);
//		peripheralPrintf(PERIPHERAL_USB, " Z:\t");
//		peripheralPrintf(PERIPHERAL_USB, "%d",data[2]);

	  }
}

/**
  * @brief  Function implementing the TempRead thread.
  * @param  argument: Not used
  * @retval None
*/
void StartTempRead(void const * argument)
{
	for(;;)
	{
	  HAL_ADC_Start(&hadc1);
	  if((HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) || getValueADC==true)    // get value of temperature if ADC it's work
	  {
		  getValueADC=true;
		  ValueLM35=HAL_ADC_GetValue(&hadc1);  // read value of temp

		  int8_t ValueLM35_Conv=ValueLM35*0.5;
		  ValueLM35= ValueLM35_Conv;

		  HAL_ADC_Stop(&hadc1);  // stop read
		  peripheralPrintf(PERIPHERAL_USB, "T:\t");
		  peripheralPrintf(PERIPHERAL_USB, "%d",ValueLM35_Conv);
		  peripheralPrintf(PERIPHERAL_USB, "\n");
		  osDelay(300);
	  }
	  else  													// if ADC it's not working
	  {
		  ValueLM35=0;
	  }
	  HAL_Delay(100);
	}
}

/**
  * @brief  Function implementing ----------------
  * @param  argument: Not used
  * @retval None
*/
void SetPoint(void const * argument)
{
	for(;;)
	{
		 setpoint=(int8_t)atoi((char*)received_data);
		 //setpoint=20;
		 osDelay(1);

		  if(setpoint!=0)
		  {
			  //peripheralPrintf(PERIPHERAL_USB, "calcular");
			  osDelay(1);
			  osMessagePut(myQueue01Handle, 1000,100);
		  }

	}
}

/**
  * @brief  Function implementing the PID.
  * @param  argument: PID input and setpoint
  * @retval None
*/
void PID(int8_t PIDinput, int8_t PIDsetpoint)
{
		kp=2;
		ki=5;
		kd=2;
		int8_t erro= (PIDsetpoint- PIDinput);
		//uint8_t erro_d= (PIDinput- PIDsetpoint);

		proporcional=kp*erro;
		integral= (integral+erro)*ki;
		derivativo=(erro-last_erro)*kd;

		int8_t pwm= proporcional+ integral+ derivativo;

		last_erro=erro;

		if(PIDinput>PIDsetpoint)    // ativa ventoinha
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,1);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD4_Pin,0);

			//PID_Compute(myPID);
			//HAL_GPIO_WritePin(GPIOA,5,myPID->myOutput);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwm);
		}
		else if(PIDinput<PIDsetpoint)   // ativa aquecimento
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,0);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD4_Pin,1);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pwm);
		}
		else
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,0);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD4_Pin,0);
			integral=0;
			pwm=0;
		}
}

/**
  * @brief  Function implementing calculate STD.
  * @param  argument: none
  * @retval None
*/
void STD()
{

	int32_t avgX=0;
	int32_t avgY=0;
	int32_t avgZ=0;

	int32_t auxX=0;
	int32_t auxY=0;
	int32_t auxZ=0;

	int16_t stdX=0;
	int16_t stdY=0;
	int16_t stdZ=0;

// calcula média para cada eixo
	avgX=sumX/10;
	avgY=sumY/10;
	avgZ=sumZ/10;

	for(int i=0; i<=9; i++)
	{
		//inicia calculo do desvio padrão
		auxX += (samplesACC_X[i]-avgX)*(samplesACC_X[i]-avgX);
		auxY += (samplesACC_Y[i]-avgY)*(samplesACC_Y[i]-avgY);
		auxZ += (samplesACC_Z[i]-avgZ)*(samplesACC_Z[i]-avgZ);

		if(i==9)
		{
			//peripheralPrintf(PERIPHERAL_USB, "x:\t");
			stdX=sqrt(auxX/9);
			peripheralPrintf(PERIPHERAL_USB, "X:\t");
			peripheralPrintf(PERIPHERAL_USB, "%d",stdX);

			stdY=sqrt(auxY/9);
			peripheralPrintf(PERIPHERAL_USB, " Y:\t");
			peripheralPrintf(PERIPHERAL_USB, "%d",stdY);

			stdZ=sqrt(auxZ/9);
			peripheralPrintf(PERIPHERAL_USB, " Z:\t");
			peripheralPrintf(PERIPHERAL_USB, "%d",stdZ);

//			if(stdX!=0)
//			{
//
//				if(stdY!=0)
//				{
//					peripheralPrintf(PERIPHERAL_USB, " Y:\t");
//					peripheralPrintf(PERIPHERAL_USB, "%d",stdY);
//
//					if(stdZ!=0)
//					{
//						peripheralPrintf(PERIPHERAL_USB, " Z:\t");
//						peripheralPrintf(PERIPHERAL_USB, "%d",stdZ);
//					}
//					else
//						peripheralPrintf(PERIPHERAL_USB, " Z:0\t");
//				}
//				else
//					peripheralPrintf(PERIPHERAL_USB, " Y:0\t");
//
//			}
//			else
//				peripheralPrintf(PERIPHERAL_USB, "X:0\t");
		}

	}
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
