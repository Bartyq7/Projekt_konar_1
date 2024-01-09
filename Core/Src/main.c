/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "mpu6050_register_map.h"
//#include "bme280_add.h"
//#include "bme280_defs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char buffer[20]={0};
uint8_t button=0;
uint8_t data_buff[64]={0};
static int16_t x=0,y=0,z=0;
static char str[64]={0};
float Ax=0, Ay=0, Az=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*int __io_putchar(int ch)
{
  if (ch == '\n') {
    __io_putchar('\r');
  }

  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

  return 1;
}
*/

void mpu6050_read(uint8_t address, uint8_t *dest, uint8_t num){
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDRESS , address, 1,dest , num, HAL_MAX_DELAY);
}

void mpu6050_write(uint8_t address, uint8_t value){
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDRESS, address, 1, &value, sizeof(value), HAL_MAX_DELAY);
}
void mpu6050_read_it(uint8_t address, uint8_t num){
	HAL_I2C_Mem_Read_IT(&hi2c1,MPU6050_I2C_ADDRESS , address, 1,data_buff , num);
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance== TIM16){
		HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
	}
	if(htim->Instance== TIM2){
		HAL_UART_Transmit_IT(&huart2,(uint8_t*) str, strlen(str));
		}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
		HAL_UART_Transmit_IT(&huart2,(uint8_t*) buffer, sizeof(buffer));
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /*
  if(BME280_init() != BME280_OK) {
  	  printf("Blad inicjalizacji!\n");
    }
  */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim16);
  sprintf(buffer,"Button pressed\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t who_am_i=0;
  	mpu6050_read(MPU6050_WHO_AM_I, &who_am_i ,1);
    if (who_am_i == 0x68) {
      // printf("Found\n");
      } else {
      // printf("Error:(0x%02X)\n", who_am_i);
      }
    mpu6050_write(MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_WAKE);

    mpu6050_write(MPU6050_SMPRT_DIV, 0x07);

    mpu6050_write(MPU6050_CONFIG, MPU6050_CONFIG_ACCEL_XOUT_L|MPU6050_CONFIG_ACCEL_YOUT_L|MPU6050_CONFIG_ACCEL_ZOUT_L);

    mpu6050_write(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_VALUE);

    mpu6050_read_it(MPU6050_ACCEL_XOUT_H, 6);
  while (1)
  {
	  	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)== 0){
	  		  HAL_Delay(5);
	  		  button=1;
	  		  while(!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin));
		  	  }
		  button=0;


		  mpu6050_read_it(MPU6050_ACCEL_XOUT_H, 6);

		  x=data_buff[0]<<8 | data_buff[1];
		  y=data_buff[2]<<8 | data_buff[3];
		  z=data_buff[4]<<8 | data_buff[5];

		  Ax=(float)x/8192.f;
		  Ay=(float)y/8192.f;
		  Az=(float)z/8192.f;



		  sprintf(str,"Ax= %.2f Ay= %.2f Az=%.2f \n\r",Ax, Ay, Az);
		  //HAL_UART_Transmit(&huart2, str, strlen(str), 100);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 13;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
