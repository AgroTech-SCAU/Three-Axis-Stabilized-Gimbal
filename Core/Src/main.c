/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "bdma.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "entry.h"
#include "stm32_hal_dwt.h"
#include "stm32_hal_uart.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* FireWater roll diagnostics, 100 Hz.
 * CH0 target-minus-encoder deg; CH1 camera roll error deg; CH2 active;
 * CH3 latched exit reason; CH4 exit count; CH5 feedback age ms;
 * CH6 validity bits (1=CH0 valid, 2=CH1 valid); CH7 UART dropped frames.
 * Invalid plotted angles are held, never replaced by a -9999 spike.
 * Target is a software command, not a motor acknowledgement.
 */
#define MAIN_ROLL_TELEMETRY_PERIOD_MS 10u
static volatile uint32_t s_main_telemetry_drop_count;
static void main_roll_vofa_task(void) {
    static uint32_t last_ms;
    uint32_t now_ms=HAL_GetTick();
    if((uint32_t)(now_ms-last_ms)<MAIN_ROLL_TELEMETRY_PERIOD_MS) return;
    last_ms=now_ms;
    BusMotorFeedback rm={0};
    uint32_t age=UINT32_MAX;
    bool ready=assemble_ht_motor_get_feedback_snapshot_age(ENTRY_GIMBAL_ROLL_MOTOR_ID,&rm,&age);
    const GimbalControl *g=&s_entry_gimbal_control;
    const GimbalStartup *a=&g->startup;
    const float deg=57.29577951308232f;
    float target=a->running?(g->roll.active?g->roll.command:NAN):
        (a->captured?a->roll_position:NAN);
    bool imu_fresh=s_entry_log_imu_seen &&
        (uint32_t)(dwt_get_us()-s_entry_imu_snapshot.sample.gyro_timestamp_us)<=10000u;
    float error=NAN;
    if(g->reference.ready && imu_fresh) {
        float e=g->reference.target.roll-s_entry_imu_snapshot.angle.roll;
        error=atan2f(sinf(e),cosf(e))*deg;
    }
    static float last_difference, last_error;
    bool difference_valid=isfinite(target) && ready && (rm.valid&BUS_MOTOR_FEEDBACK_POSITION);
    bool error_valid=isfinite(error);
    if(difference_valid) last_difference=(target-rm.position)*deg;
    if(error_valid) last_error=error;
    float channels[]={
        last_difference,last_error,g->roll.active?1.0f:0.0f,
        (float)g->roll.last_exit_reason,(float)g->roll.exit_count,
        age!=UINT32_MAX?(float)age:-1.0f,
        (float)((difference_valid?1u:0u)|(error_valid?2u:0u)),
        (float)s_main_telemetry_drop_count
    };
    char line[192];
    size_t used=5u;
    memcpy(line,"roll:",used);
    for(unsigned i=0;i<sizeof(channels)/sizeof(channels[0]);++i) {
        /* Numeric sentinel keeps all channels parseable before data is ready.
         * Integer formatting avoids newlib-nano float-printf dependency. */
        float v=channels[i];
        if(!isfinite(v) || fabsf(v)>=1.0e9f) v=-9999.0f;
        double magnitude=fabs((double)v);
        unsigned long whole=(unsigned long)magnitude;
        unsigned long frac=(unsigned long)((magnitude-whole)*1000.0+0.5);
        if(frac>=1000ul) { ++whole; frac=0; }
        int n=snprintf(line+used,sizeof(line)-used,"%s%s%lu.%03lu",
            i?",":"",signbit(v)?"-":"",whole,frac);
        if(n<0 || (size_t)n>=sizeof(line)-used) { ++s_main_telemetry_drop_count; return; }
        used+=(size_t)n;
    }
    if(used+2u>sizeof(line)) { ++s_main_telemetry_drop_count; return; }
    line[used++]='\r'; line[used++]='\n';
    if(!uart1_write_it(line,(uint32_t)used)) ++s_main_telemetry_drop_count;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();
  MX_BDMA_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_SPI6_Init();
  MX_FDCAN2_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USART10_UART_Init();
  /* USER CODE BEGIN 2 */
    entry_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while(1) {
        entry_loop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        main_roll_vofa_task();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 62;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 4096;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while(1) {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
