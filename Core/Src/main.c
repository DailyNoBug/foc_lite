/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BLDCMotor.h"
#include "FOCMotor.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define AS5600_I2C_HANDLE hi2c1
#define I2C_TIME_OUT_BASE   10
#define I2C_TIME_OUT_BYTE   1
#define AS5600_RAW_ADDR    0x36
#define AS5600_ADDR        (AS5600_RAW_ADDR << 1)
#define AS5600_WRITE_ADDR  (AS5600_RAW_ADDR << 1)
#define AS5600_READ_ADDR   ((AS5600_RAW_ADDR << 1) | 1)
#define AS5600_RESOLUTION 4096 //12bit Resolution
#define AS5600_RAW_ANGLE_REGISTER  0x0C

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int i2cWrite(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
    status = HAL_I2C_Master_Transmit(&AS5600_I2C_HANDLE, dev_addr, pData, count, i2c_time_out);
    return status;
}
static int i2cRead(uint8_t dev_addr, uint8_t *pData, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;
    status = HAL_I2C_Master_Receive(&AS5600_I2C_HANDLE, (dev_addr | 1), pData, count, i2c_time_out);
    return status;
}
uint16_t AS5600_ReadAngle(void) {
    uint16_t raw_angle;
    uint8_t buffer[2] = {0};
    uint8_t raw_angle_register = AS5600_RAW_ANGLE_REGISTER;
    i2cWrite(AS5600_ADDR, &raw_angle_register, 1);
    i2cRead(AS5600_ADDR, buffer, 2);
    raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    return raw_angle;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define RX_BUFFER_SIZE 64
extern uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t value[RX_BUFFER_SIZE];
uint8_t ans[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart3,(uint8_t *)value,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char buffer[1024];
  uint16_t ADC_Value=0;
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
//  HAL_TIM_OC_Start(&htim1,TIM_CHANNEL_1);
//  HAL_TIM_OC_Start(&htim1,TIM_CHANNEL_2);
//  HAL_TIM_OC_Start(&htim1,TIM_CHANNEL_3);

    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    voltage_power_supply=12;   //V
    voltage_limit=6.9;           //V?????????12/1.732=6.9????????????????0.5-1????????????????1-3
    velocity_limit=100;         //rad/s angleOpenloop() use it
    controller=Type_velocity_openloop;  //Type_angle_openloop;  //
    pole_pairs=12;              //????
  while (1)
  {
//      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
//      HAL_Delay(100);
//      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
//      HAL_Delay(100);
//      memset(buffer,0,sizeof (buffer));
//      HAL_ADC_Start(&hadc2);
//      ADC_Value = HAL_ADC_GetValue(&hadc2);
//      sprintf(buffer,"ADC2: %d\n",ADC_Value);
//      CDC_Transmit_FS(buffer,strlen(buffer));
//      memset(buffer,0,sizeof (buffer));
//      HAL_ADC_Start(&hadc1);
//      ADC_Value = HAL_ADC_GetValue(&hadc1);
//      sprintf(buffer,"ADC1: %d\n",ADC_Value);
//      CDC_Transmit_FS(buffer,strlen(buffer));

//      uint16_t angle = AS5600_ReadAngle();
//      memset(buffer,0,sizeof(buffer));
//      sprintf(buffer,"angle: %d\n",angle);
//      CDC_Transmit_FS(buffer, strlen(buffer));
      uint8_t x=0;
//        if(strlen(ans)){
//            if(ans[0]=='l'){
//                x=0;
//                for(int i=1;i< strlen(ans);i++){
//                    if(ans[i]>='0'&&ans[i]<='9'){
//                        x=x*10+(ans[i]-'0');
//                    }
//                }
//                double pr=10.0*((double)x/1024.0);
//                memset(buffer,0,sizeof(buffer));
//                sprintf(buffer,"res: %s : %d\n",ans,x);
//                CDC_Transmit_FS(buffer, strlen(buffer));
//                move(pr);
//            }else if(ans[0]=='r'){
//                x=0;
//                for(int i=1;i< strlen(ans);i++){
//                    if(ans[i]>='0'&&ans[i]<='9'){
//                        x=x*10+(ans[i]-'0');
//                    }
//                }
//                double pr=-10.0*((double)x/1024.0);
//                memset(buffer,0,sizeof(buffer));
//                sprintf(buffer,"res: %s : %d :%lf\n",ans,x,pr);
//                CDC_Transmit_FS(buffer, strlen(buffer));
//                move(pr);
//            }
//        }

move(10);
//      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);

//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
//      HAL_Delay(5);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
//      HAL_Delay(5);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
//      HAL_Delay(5);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
//      HAL_Delay(5);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
//      HAL_Delay(5);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
//      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
//      HAL_Delay(5);

//      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
