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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct Frame {
    float fdata[6];
    unsigned char tail[4];
} Frame_t;

typedef struct SOGI{
    float ualfa_0;
    float SOGI_Ualfa;
    float SOGI_Ubeta;

    float integral_2;
    float integral_3;

    float Ugird_W0;
    float samp_t;
} SOGI_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float ek;
    float ek_1;
    float ek_2;
    float uk;
    float uk_1;
} PID_t;

typedef struct {
    float wt;
    float w0;
    PID_t pid;
    SOGI_t sogi;
} PLL_t;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float ek;
    float ek1;
    float ek2;
    float location_sum;
    float out;
}PID_LocTypeDef;

typedef struct{
    float kp ;
    float kr ;
    float wi ;
    float reference ;
    float ts ;
    float L_vir;
    float output_of_backward_integrator ;
    float output_of_feedback ;
    float output_of_forward_integrator ;
    float last_input_of_forward_integrator ;
    float error;
    float input_of_forward_integrator;
    float output ;
}PR_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Fz        50.0f
#define PAID      3.14157f
#define T_sample  0.00005f
#define L_val     0.002f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t dma_adc_buffer[3];
volatile float U0, I0, Ud;

float ud, uq, id, iq, detad, detaq, cd, cq, ca, cb, Ud_AIM;

PLL_t U0_PLL;

SOGI_t I0_SOGI;
PR_t I0_PR;

PID_LocTypeDef Ud_PI;
PID_LocTypeDef Id_PI;
PID_LocTypeDef Iq_PI;

float SinVal, CosVal;

float m, n;
uint16_t count;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Float_Limit(float* num, float min, float max);
void arm_park_f32 (float alpha, float beta, float* id, float* iq, float sinVal, float cosVal);
void arm_inv_park_f32 (float id, float iq, float* alpha, float* beta, float sinVal, float cosVal);

void UART_SendFrame(UART_HandleTypeDef *huart, float ch1,float ch2,float ch3, float ch4,float ch5,float ch6);

void PLL_init(PLL_t *pll);
void Sogi_init(SOGI_t *sogi);
void PLL_update(PLL_t *pll, float ualpha_input, float* sinVal, float* cosVal);
void Sogi_fun(SOGI_t *sogi, float ualpha_input);
void dq_pll(PLL_t *pll);
float zl_pid_increase(float error, PID_t *pid);

void PID_Init(PID_LocTypeDef *PID,float kp,float ki,float kd);
float PID_increment(float setvalue, float actualvalue, float PID_LIMIT_MIN, float PID_LIMIT_MAX, PID_LocTypeDef *PID);

void PR_Init(PR_t *s, float kp_set, float kr_set, float wi_set, float ts);
void PR_calc(PR_t *s, float reference, float feedback, float wg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  // PR_Init(&I0_PR, 0.01, 10, 2, T_sample);

  PLL_init(&U0_PLL);
  // Sogi_init(&I0_SOGI);

  // PID_Init(&Ud_PI,0.004,0.0015,0);
  // PID_Init(&Id_PI,0.5,0.0001,0);
  // PID_Init(&Iq_PI,0.5,0.0001,0);
  // Ud_AIM = 36.0f;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  // HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  // HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  HAL_ADC_Start_DMA(&hadc1,dma_adc_buffer,3);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_TC);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {
    U0 = (((dma_adc_buffer[0] / 4095.0f) * 3.3f) - 1.5f) * 30.0f;
    I0 = (((dma_adc_buffer[1] / 4095.0f) * 3.3f) - 1.5f) * 3.0f;
    Ud = (dma_adc_buffer[2] / 4095.0f) * 3.3f * 20.0f;

    // PLL_update(&U0_PLL, U0, &SinVal, &CosVal);



    // Sogi_fun(&I0_SOGI, I0);

    // count++;
    // if (count >= 200)
    // {
    //   count = 0;
    //   m -= PID_increment(Ud_AIM * 1.414f, Ud, -0.05,0.05, &Ud_PI);
    //   Float_Limit(&m, 0.55f, 0.9f);
    // }

    // arm_park_f32(U0_PLL.sogi.SOGI_Ualfa, U0_PLL.sogi.SOGI_Ubeta, &ud, &uq, SinVal, CosVal);
    // arm_park_f32(I0_SOGI.SOGI_Ualfa, I0_SOGI.SOGI_Ubeta, &id, &iq, SinVal, CosVal);

    // detad -= PID_increment(10, id, -0.02, 0.02, &Id_PI);
    // detaq += PID_increment(0, iq, -0.02, 0.02, &Iq_PI);

    // cd = (ud  -detad + 100 * PAID * L_val * iq) / Ud_AIM;
    // cq = (uq - detaq - 100 * PAID * L_val * id) / Ud_AIM;
    // Float_Limit(&cd, -1, 1);
    // Float_Limit(&cq, -1, 1);
    
    // arm_inv_park_f32(cd, cq, &ca, &cb, SinVal, CosVal);

    // n = m * ca;
    // Float_Limit(&n, -1, 1);

    // if (n > 0)
    // {
    //   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, n * 8400);
		//   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    // }
    // else
    // {
    //   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		//   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -n * 8400);
    // }
  }

  if (htim == &htim3)
  {
    UART_SendFrame(&huart2, U0, I0 / 3.0f, 0, 0, 0, 0);
  }
}

void Float_Limit(float* num, float min, float max)
{
  if (*num > max) *num = max;
  if (*num < min) *num = min;
}

void arm_park_f32 (float alpha, float beta, float* id, float* iq, float sinVal, float cosVal)
{
  *id =  alpha * cosVal + beta * sinVal;
  *iq = -alpha * sinVal + beta * cosVal;
}

void arm_inv_park_f32 (float id, float iq, float* alpha, float* beta, float sinVal, float cosVal)
{
  *alpha = id * cosVal - iq * sinVal;
  *beta  = id * sinVal + iq * cosVal;
}

void UART_SendFrame(UART_HandleTypeDef *huart, float ch1,float ch2,float ch3, float ch4,float ch5,float ch6)
{
    Frame_t frame;

    frame.fdata[0] = ch1;
    frame.fdata[1] = ch2;
    frame.fdata[2] = ch3;
    frame.fdata[3] = ch4;
    frame.fdata[4] = ch5;
    frame.fdata[5] = ch6;

    // 设置帧尾（VOFA+默认识别的帧尾）
    frame.tail[0] = 0x00;
    frame.tail[1] = 0x00;
    frame.tail[2] = 0x80;
    frame.tail[3] = 0x7F;

    HAL_UART_Transmit(huart, (uint8_t *)&frame, sizeof(Frame_t), HAL_MAX_DELAY);
}

void PLL_init(PLL_t *pll)
{
    pll->wt = 0;
    pll->w0 = 2*Fz*PAID; // 初始频率
    pll->pid.kp = 5;
    pll->pid.ki = 0.02;
    pll->pid.kd = 0;
    pll->pid.uk = 0;
    pll->pid.ek = pll->pid.ek_1 = pll->pid.ek_2 = 0;

    Sogi_init(&pll->sogi);
}

void Sogi_init(SOGI_t *sogi)
{
	sogi->Ugird_W0=2*Fz*PAID;
    sogi->integral_2 = 0;
    sogi->integral_3 = 0;
    sogi->samp_t = T_sample;
}

void PLL_update(PLL_t *pll, float ualpha_input, float* sinVal, float* cosVal)
{
    Sogi_fun(&pll->sogi, ualpha_input);
    dq_pll(pll);
    *sinVal = sinf(pll->wt);
    *cosVal = cosf(pll->wt);
}

void Sogi_fun(SOGI_t *sogi, float ualpha_input)
{
    sogi->ualfa_0 = ualpha_input;

    float x_a = (sogi->ualfa_0 - sogi->integral_2) * 1.0f;
    sogi->integral_2 += sogi->Ugird_W0 * (x_a - sogi->integral_3) * sogi->samp_t;
    sogi->SOGI_Ualfa = sogi->integral_2;


    sogi->integral_3 += sogi->Ugird_W0 * sogi->integral_2 * sogi->samp_t;
    sogi->SOGI_Ubeta = sogi->integral_3;
}

void dq_pll(PLL_t *pll)
{
    float uq1 = cosf(pll->wt) * pll->sogi.SOGI_Ubeta
             - sinf(pll->wt) * pll->sogi.SOGI_Ualfa;


    float con_increase = zl_pid_increase(0-uq1, &pll->pid);
    pll->w0 = 2*Fz*PAID - con_increase;
    pll->wt += pll->w0 * pll->sogi.samp_t;
    if (pll->wt >= 2 * PAID ) pll->wt -= 2 * PAID ;
}

float zl_pid_increase(float error, PID_t *pid)
{
    pid->ek = error;
    float delta_u = pid->kp * (pid->ek - pid->ek_1)
                  + pid->ki * pid->ek
                  + pid->kd * (pid->ek - 2 * pid->ek_1 + pid->ek_2);
    pid->ek_2 = pid->ek_1;
    pid->ek_1 = pid->ek;
    pid->uk += delta_u;
    return pid->uk;
}

void PID_Init(PID_LocTypeDef *PID,float kp,float ki,float kd)
{
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->ek = 0;
    PID->ek1 = 0;
    PID->ek2 = 0;
    PID->location_sum = 0;
    PID->out = 0;
}

float PID_increment(float setvalue, float actualvalue, float PID_LIMIT_MIN, float PID_LIMIT_MAX, PID_LocTypeDef *PID)
{
	PID->ek =setvalue-actualvalue;
  PID->out=PID->kp*(PID->ek-PID->ek1)+PID->ki*PID->ek+PID->kd*(PID->ek-2*PID->ek1+PID->ek2);
  PID->ek2 = PID->ek1;
  PID->ek1 = PID->ek;
	if(PID->out<PID_LIMIT_MIN)	PID->out=PID_LIMIT_MIN;
	if(PID->out>PID_LIMIT_MAX)	PID->out=PID_LIMIT_MAX;

	return PID->out;
}

void PR_Init(PR_t *s, float kp_set, float kr_set, float wi_set, float ts)
{
    s->kp = kp_set ;
    s->kr = kr_set ;
    s->wi = wi_set ;
    s->ts = ts ;
    s->L_vir=0;
    s->output_of_feedback = 0;
    s->output_of_backward_integrator = 0;
    s->output_of_forward_integrator = 0 ;
    s->error=0;
    s->input_of_forward_integrator=0;
    s->reference = 0 ;
    s->output= 0 ;
}

void PR_calc(PR_t *s, float reference, float feedback, float wg)
{
    s->reference = reference;

    s->error = reference - feedback ;
    s->input_of_forward_integrator = 2 * s->wi * s->kr * s->error - s->output_of_feedback;
    // Forward integrator :
    s->output_of_forward_integrator += s->ts *  s->input_of_forward_integrator;

    // Backward integrator:
    s->output_of_backward_integrator += s->ts * s->output_of_forward_integrator * wg * wg ;

    s->output_of_feedback = s->output_of_backward_integrator + 2 * s->wi * s->output_of_forward_integrator ;

    s->output=s->output_of_forward_integrator + s->kp* s->error;
}
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
