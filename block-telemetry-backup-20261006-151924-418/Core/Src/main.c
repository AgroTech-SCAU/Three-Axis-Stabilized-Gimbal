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
#include "chassis_yaw_hold.h"
#include "stm32_hal_dwt.h"
#include "stm32_hal_uart.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_YAW_VOFA_ENABLE 1
#define MAIN_TELEMETRY_HZ 30u

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

/* Section 8, fixed channel order. Observation only; no additional sensor reads.
 * timestamp_us is DWT microseconds modulo 2^32 (wrap after ~71.6 minutes).
 * Inputs are latest cached values, not simultaneous hardware samples.
 */
static volatile uint32_t s_main_telemetry_drop_count;

/* Integer formatting avoids requiring newlib-nano floating printf support. */
static int main_telemetry_float(char* dst, size_t size, float value) {
    if(!isfinite(value) || fabsf(value) >= 1.0e9f)
        return snprintf(dst, size, "nan");
    double magnitude = value < 0.0f ? -(double)value : (double)value;
    unsigned long whole = (unsigned long)magnitude;
    unsigned long fraction = (unsigned long)((magnitude - whole) * 1000000.0 + 0.5);
    if(fraction >= 1000000ul) { whole++; fraction = 0ul; }
    return snprintf(dst, size, "%s%lu.%06lu", value < 0.0f ? "-" : "", whole, fraction);
}

static void main_telemetry_motor(float* values, const BusMotorFeedback* fb, bool fresh) {
    if(!fresh) return;
    if(fb->valid & BUS_MOTOR_FEEDBACK_POSITION) values[0] = fb->position;
    if(fb->valid & BUS_MOTOR_FEEDBACK_VELOCITY) values[1] = fb->velocity;
    if(fb->valid & BUS_MOTOR_FEEDBACK_TORQUE) values[2] = fb->torque;
}

static void main_yaw_vofa_task(void) {
#if MAIN_YAW_VOFA_ENABLE
    static bool started;
    static uint32_t last_us, phase, seq;
    if(!init_ok) return;
    uint32_t now = dwt_get_us();
    if(!started) { last_us = now; started = true; return; }
    uint32_t elapsed = now - last_us;
    last_us = now;
    /* Rational 30 Hz scheduler: retains fractional period; no catch-up burst. */
    uint64_t credit = (uint64_t)phase + (uint64_t)elapsed * MAIN_TELEMETRY_HZ;
    if(credit < 1000000u) { phase = (uint32_t)credit; return; }
    uint32_t slots = (uint32_t)(credit / 1000000u);
    phase = (uint32_t)(credit % 1000000u);
    seq += slots;
    s_main_telemetry_drop_count += slots - 1u;

    /* v[0] is channel I2; I0/I1 are printed as exact unsigned integers. */
    float v[34];
    for(unsigned i = 0; i < 34u; ++i) v[i] = NAN;
    bool imu_fresh = s_entry_log_imu_seen &&
        (uint32_t)(HAL_GetTick() - s_entry_log_imu_ms) <= 100u;
    if(imu_fresh) {
        ImuAcc a = imu_get_acc();
        ImuGyro g = imu_get_gyro(), c = imu_get_gyro_corrected();
        ImuAngle angle = imu_get_angle();
        v[0]=a.x; v[1]=a.y; v[2]=a.z;
        v[3]=g.x; v[4]=g.y; v[5]=g.z;
        v[6]=c.x; v[7]=c.y; v[8]=c.z;
        v[9]=angle.roll; v[10]=angle.pitch; v[11]=angle.yaw;
#if ENTRY_GIMBAL_ROLL_ENABLE
        if(s_entry_gimbal_zero_ready) {
            v[12]=angle.roll-s_entry_gimbal_zero_roll_angle;
            v[13]=s_entry_gimbal_roll_error_filter;
        }
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
        if(s_entry_gimbal_zero_ready) {
            v[14]=angle.pitch-s_entry_gimbal_zero_pitch_angle;
            v[15]=s_entry_gimbal_pitch_error_filter;
        }
#endif
#if ENTRY_GIMBAL_YAW_ENABLE
        if(s_entry_gimbal_yaw_ref_ready)
            v[16]=entry_wrap_pi(angle.yaw-s_entry_gimbal_imu_ref_yaw);
#endif
    }
#if ENTRY_GIMBAL_ROLL_ENABLE
    if(s_entry_gimbal_zero_ready) v[17]=s_entry_gimbal_roll_target;
#endif
#if ENTRY_GIMBAL_PITCH_ENABLE
    if(s_entry_gimbal_zero_ready) v[18]=s_entry_gimbal_pitch_target;
#endif
#if ENTRY_GIMBAL_YAW_ENABLE
    if(s_entry_gimbal_yaw_ref_ready) v[19]=s_entry_gimbal_yaw_target;
#endif
    BusMotorFeedback roll = {0}, pitch = {0}, yaw = {0};
    bool rf=assemble_ht_motor_get_feedback_snapshot(ENTRY_GIMBAL_ROLL_MOTOR_ID, &roll);
    bool pf=assemble_ht_motor_get_feedback_snapshot(ENTRY_GIMBAL_PITCH_MOTOR_ID, &pitch);
    bool yf=assemble_dm_motor_get_feedback_snapshot(&yaw);
    main_telemetry_motor(&v[20], &roll, rf);
    main_telemetry_motor(&v[23], &pitch, pf);
    main_telemetry_motor(&v[26], &yaw, yf);
    VisionLevelResult vision;
    if(vision_comms_get_result(&vision) &&
       vision_comms_result_is_fresh(ENTRY_VISION_RESULT_TIMEOUT_MS)) {
        v[29]=vision.corr_x_deg; v[30]=vision.corr_y_deg;
    }
    v[31]=rf; v[32]=pf; v[33]=yf;

    /* One atomic nonblocking enqueue; a full UART queue drops the whole frame. */
    static const char* const names[34] = {
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_raw_x",
        "gyro_raw_y",
        "gyro_raw_z",
        "gyro_corrected_x",
        "gyro_corrected_y",
        "gyro_corrected_z",
        "imu_roll",
        "imu_pitch",
        "imu_yaw",
        "roll_error_raw",
        "roll_error_filtered",
        "pitch_error_raw",
        "pitch_error_filtered",
        "yaw_delta",
        "roll_target",
        "pitch_target",
        "yaw_target",
        "roll_pos",
        "roll_vel",
        "roll_torque",
        "pitch_pos",
        "pitch_vel",
        "pitch_torque",
        "yaw_pos",
        "yaw_vel",
        "yaw_torque",
        "vision_corr_x",
        "vision_corr_y",
        "roll_feedback_fresh",
        "pitch_feedback_fresh",
        "yaw_feedback_fresh",
    };
    static char line[1536];
    int len=snprintf(line, sizeof(line), "gimbal timestamp_us:%lu, seq:%lu", (unsigned long)now, (unsigned long)seq);
    if(len < 0 || (size_t)len >= sizeof(line)) goto drop;
    size_t used=(size_t)len;
    for(unsigned i=0; i<34u; ++i) {
        len=snprintf(line+used, sizeof(line)-used, ", %s:", names[i]);
        if(len < 0 || (size_t)len >= sizeof(line)-used) goto drop;
        used+=(size_t)len;
        if(i >= 31u)
            len=snprintf(line+used, sizeof(line)-used, "%u", v[i] != 0.0f ? 1u : 0u);
        else
            len=main_telemetry_float(line+used, sizeof(line)-used, v[i]);
        if(len < 0 || (size_t)len >= sizeof(line)-used) goto drop;
        used+=(size_t)len;
    }
    if(used+2u >= sizeof(line)) goto drop;
    line[used++]='\r'; line[used++]='\n'; line[used]='\0';
    if(uart1_write_it(line, (uint32_t)used)) return;
drop:
    s_main_telemetry_drop_count++;
#endif
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
        main_yaw_vofa_task();
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
