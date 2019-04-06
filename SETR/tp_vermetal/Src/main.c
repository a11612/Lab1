
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdarg.h"
#include "stdbool.h"
#include <PID_v1.h> //include da library PID
#include "usbd_cdc_if.h"
#include "../../BSP/STM32F411E-Discovery/stm32f411e_discovery_accelerometer.h"  //include para aceder ao ACC

#include <math.h>

// variáveis PID necessárias para implementar library
double Setpoint, Input, Output;
PidType* myPID;
FloatType myPIDInput=12;
FloatType myPIDOutput;
FloatType myPIDSetpoint=6;
FloatType Kp=5, Ki=2, Kd=3;

// variáveis PID criadas para  simular a utilização do PID
int8_t proporcional=0;
int8_t integral= 0;
int8_t derivativo=0;
int8_t kp,ki,kd;
int8_t last_erro=0;

int16_t samplesACC_X[10];
int16_t samplesACC_Y[10];
int16_t samplesACC_Z[10];


bool getValueADC=false;   // variável auxiliar para ler o ADC

int16_t data[3];  // variável auxiliar para ler o ACC

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

ADC_HandleTypeDef hadc1;  //Structure definition to ADC

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

enum
 {
 	PERIPHERAL_USART,	// UART serial device
 	PERIPHERAL_USB 		// USB serial device
 };

/**
  * @brief  Function implementing the Printf for COM
  * @param  argument: Not used
  * @retval None
*/
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


/**
  * @brief  Função para ler os valores ACC e retorna para o main os valores
  * @param  argument: Not used
  * @retval Array [3] Values of ACC , é uma variável inteiro de 2 bytes que permite valores positivos e negativos [ex: data[1]=; data[2]=; data[3]= ]
*/
uint16_t* ReadACC()
{
	int32_t sumX=0;
	int32_t sumY=0;
	int32_t sumZ=0;

	int32_t avgX=0;
	int32_t avgY=0;
	int32_t avgZ=0;

	int32_t auxX=0;
	int32_t auxY=0;
	int32_t auxZ=0;

	int16_t stdX=0;
	int16_t stdY=0;
	int16_t stdZ=0;
	//ler os valores adc e coloca na variável data os valores

	for(int i=0; i<=9; i++)
	{
		 // adquire 10 amostra do ACC
		BSP_ACCELERO_GetXYZ(data);
		samplesACC_X[i]=data[0];
		samplesACC_Y[i]=data[1];
		samplesACC_Z[i]=data[2];

		// somatorio das amostras recolhidas
		sumX=sumX+ samplesACC_X[i];
		sumY=sumY+ samplesACC_Y[i];
		sumZ=sumZ+ samplesACC_Z[i];

		//quando recolher 10 amostras
		if(i==9)
		{
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
					stdX=sqrt(auxX/9);
					stdY=sqrt(auxY/9);
					stdZ=sqrt(auxZ/9);
				}
			}
		}
	}

	// no final retorna os valores do desvio padrão
	data[0]=stdX;
	data[1]=stdY;
	data[2]=stdZ;

	return &data;  // retorna os valores lidos para a função main()
}

/**
  * @brief  Função para ler os valores de temperatura apartir de uma entrada ADC
  * @param  argument: Not used
  * @retval Retorna um valor de temperatura que será um inteiro  de 1 byte [ex: adc_value=],
  * caso não seja possivel aceder ao ADC retorna um valor Nulo [adc_value=0]
*/
uint16_t ReadLM35()
{
		  HAL_ADC_Start(&hadc1);    //sempre que necessita de efetuar uma nova leitura, é necessário dar um pulso

		  int8_t adc_value=0;

		  if((HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) || getValueADC==true)    //se o ADC estiver funcional
		  {
			  getValueADC=true;
			  adc_value=HAL_ADC_GetValue(&hadc1);  // lê o valor de temperatura

			  HAL_ADC_Stop(&hadc1);  // e no fim, faz stop do ADC
		  }
		  else  	 // caso o ADC não dê uma resposta OK de funcionamento												// if ADC it's not working
		  {
			  adc_value=0;

			  return 0;		// return value 0
		  }


    	  return adc_value;
}

/**
  * @brief  Função para implementar o PID
  * @param  argument: Os parametros da função são o valor de temperatura atual e o valor de setpoint recebido através da porta COM,
  * ambos os valores são inteiros de 1 byte e podem ser negativos ou positivos
  * @retval none
*/
void PID(int8_t PIDinput, int8_t PIDsetpoint)
{

	//	PID_init(myPID,Kp,Ki,Kd, PID_Direction_Direct);
	//	PID_SetMode(myPID, PID_Mode_Automatic);
	//
	//	myPID->myInput= PIDinput;
	//	myPID->mySetpoint=PIDsetpoint;
	//	myPID->myInput=myPIDInput;
	//	myPID->mySetpoint=myPIDSetpoint;

		kp=2; // ganho proporcional
		ki=5; //ganho integral
		kd=2; // ganho derivativo
		int8_t erro= (PIDsetpoint- PIDinput); // calculo do erro (diferença entre o valor atual de temperatura e o valor de setpoint)
		//uint8_t erro_d= (PIDinput- PIDsetpoint);


      // calculo de PID
		proporcional=kp*erro;
		integral= (integral+erro)*ki;
		derivativo=(erro-last_erro)*kd;

		int8_t pwm= proporcional+ integral+ derivativo;

		last_erro=erro;

		if(PIDinput>PIDsetpoint)    // caso a temperatura atual seja superior ao setpoint (está mais quente doque o desejado), ativa a ventoinha
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,1);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD4_Pin,0);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,pwm);
		}
		else if(PIDinput<PIDsetpoint)   // caso a temperatura atual seja inferior ao setpoint (está mais frio doque o desejado), ativa a lâmpada
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,0);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD4_Pin,1);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,pwm);
		}
		else    // caso a temperatura atual coincida com o valor de setpoint, não executa nenhuma tarefa
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin,0);
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD4_Pin,0);
			integral=0;
			pwm=0;
		}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

    BSP_ACCELERO_Init();	  // faz init do ACC

    //local variables
    uint16_t* ValueACC={0,0,0};
    int8_t ValueLM35=0;

   	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  //start PWM ventoinha
   	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  //start PWM lampada
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)   //infinite loop
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  ValueACC=ReadACC();
	  HAL_Delay(100); // wait 100ms

	  ValueLM35=ReadLM35();   //*5/(1023))/0.01   -32)/1.8
	  int8_t ValueLM35_Conv=ValueLM35*0.5;

	  HAL_Delay(100); // wait 100ms

	  // valores hardcode para testar funcionamento do PID
	  //uint8_t setpoint=10;
	  //ValueLM35_Conv=23;
	  //PID(ValueLM35_Conv, setpoint);
	  int8_t setpoint = (int8_t)atoi((char*)received_data);
	  if(setpoint!=0)  // quando recebe alguma coisa na porta COM
	  {
		  //int8_t setpoint = (int8_t)atoi((char*)received_data);  //atribui o valor que recebeu à variavel setpoint
		  PID(ValueLM35_Conv, setpoint); //executa a função PID
	  }
	  //if setpoint is different of zero
	  if(USB_ReceiveString()==1)  // quando recebe alguma coisa na porta COM
	  {
		  int8_t setpoint = (int8_t)atoi((char*)received_data);  //atribui o valor que recebeu à variavel setpoint
		  PID(ValueLM35_Conv, setpoint); //executa a função PID
	  }
	  // else skip pid function

	  // imprime na porta COM os valores obtidos nas leituras dos sensores
		  peripheralPrintf(PERIPHERAL_USB, "X:\t");
		  peripheralPrintf(PERIPHERAL_USB, "%d",ValueACC[0]);
		  peripheralPrintf(PERIPHERAL_USB, " Y:\t");
		  peripheralPrintf(PERIPHERAL_USB, "%d",ValueACC[1]);
		  peripheralPrintf(PERIPHERAL_USB, " Z:\t");
		  peripheralPrintf(PERIPHERAL_USB, "%d",ValueACC[2]);
		  peripheralPrintf(PERIPHERAL_USB, " T:\t");
		  peripheralPrintf(PERIPHERAL_USB, "%d",ValueLM35_Conv);
		  peripheralPrintf(PERIPHERAL_USB, "\n");
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
