/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Self-balancing dual motor control)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define MPU6050_ADDR         (0x68 << 1)
#define SMPLRT_DIV_REG       0x19
#define PWR_MGMT_1_REG       0x6B
#define WHO_AM_I_REG         0x75
#define ACCEL_XOUT_H_REG     0x3B
/* USER CODE END PD */

/* USER CODE BEGIN PV */
int16_t raw_ax, raw_az, raw_gy;
float accel_angle, gyro_rate;
float Current_Angle = 0.0f;
float Gyro_Y = 0.0f;

float Kp = 22.0f;
float Ki = 0.0f;
float Kd = 1.2f;

float Target_Angle = 0.0f;
float Error = 0.0f;
float Integrated_Error = 0.0f;
int16_t Motor_PWM = 0;
/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
uint8_t MPU6050_Init(void)
{
  uint8_t check, data;

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
  if (check == 0x68)
  {
    data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);
    return 0;
  }

  return 1;
}

void MPU6050_Read_Raw(int16_t *Accel_X, int16_t *Accel_Z, int16_t *Gyro_Y)
{
  uint8_t Rec_Data[14];

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, 1000);
  *Accel_X = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  *Accel_Z = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
  *Gyro_Y  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
}

int16_t PWM_Limit(int16_t pwm)
{
  if (pwm > 850) return 850;
  if (pwm < -850) return -850;
  return pwm;
}

int16_t Balance_PID(float Angle, float Gyro)
{
  Error = Angle - Target_Angle;
  Integrated_Error += Error;

  if (Integrated_Error > 2000) Integrated_Error = 2000;
  if (Integrated_Error < -2000) Integrated_Error = -2000;

  float pwm_out = (Kp * Error) + (Ki * Integrated_Error) + (Kd * Gyro);
  return (int16_t)pwm_out;
}

void Motor_Set(int16_t left_pwm, int16_t right_pwm)
{
  left_pwm = PWM_Limit(left_pwm);
  right_pwm = PWM_Limit(right_pwm);

  if (left_pwm >= 0)
  {
    int16_t final_pwm = left_pwm + 80;
    if (final_pwm > 850) final_pwm = 850;

    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, final_pwm);
  }
  else
  {
    int16_t final_pwm = -left_pwm + 80;
    if (final_pwm > 850) final_pwm = 850;

    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, final_pwm);
  }

  if (right_pwm >= 0)
  {
    int16_t final_pwm = right_pwm + 80;
    if (final_pwm > 850) final_pwm = 850;

    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, final_pwm);
  }
  else
  {
    int16_t final_pwm = -right_pwm + 80;
    if (final_pwm > 850) final_pwm = 850;

    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, final_pwm);
  }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  MPU6050_Init();

  while (1)
  {
    MPU6050_Read_Raw(&raw_ax, &raw_az, &raw_gy);

    accel_angle = atan2((float)raw_ax, (float)raw_az) * 57.29578f;
    gyro_rate = (float)raw_gy / 131.0f;
    Current_Angle = 0.98f * (Current_Angle + gyro_rate * 0.005f) + 0.02f * accel_angle;
    Gyro_Y = gyro_rate;

    if (Current_Angle > 45.0f || Current_Angle < -45.0f)
    {
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      Integrated_Error = 0;
    }
    else
    {
      Motor_PWM = Balance_PID(Current_Angle, Gyro_Y);
      Motor_Set(Motor_PWM, Motor_PWM);
    }

    HAL_Delay(5);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
