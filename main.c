#include "main.h"
#include "stm32f1xx_hal.h"
// Servo Configuration
#define SERVO_GPIO_PORT GPIOA
#define SERVO_GPIO_PIN GPIO_PIN_6
#define SERVO_TIM_INSTANCE TIM3
#define SERVO_TIM_CHANNEL TIM_CHANNEL_1
#define SERVO2_GPIO_PORT GPIOA
#define SERVO2_GPIO_PIN GPIO_PIN_7
#define SERVO2_TIM_INSTANCE TIM3
#define SERVO2_TIM_CHANNEL TIM_CHANNEL_2
#define SERVO3_GPIO_PORT GPIOB
#define SERVO3_GPIO_PIN GPIO_PIN_0
#define SERVO3_TIM_INSTANCE TIM3
#define SERVO3_TIM_CHANNEL TIM_CHANNEL_3
#define SERVO_MIN_PULSE 0.65f  // Minimum Pulse Width in ms
#define SERVO_MAX_PULSE 2.3f   // Maximum Pulse Width in ms
#define SERVO_PWM_FREQ 50      // PWM Frequency in Hz (20ms period)
/* Private includes ----------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
uint16_t min_pulse_ticks;
uint16_t max_pulse_ticks;
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_ConfigS(void);
void SystemClock_ConfigL(void);
static void MX_GPIO_Init_Servo(void);
static void MX_GPIO_Init_LFR(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void SERVO_Init(void);
void SERVO_MoveTo(float angle);
void SERVO2_MoveTo(float angle);
void SERVO3_MoveTo(float angle);
void SERVO_Sweep(void);
int main(void)
{
 /* USER CODE BEGIN 1 */
 /* USER CODE END 1 */
 /* MCU Configuration--------------------------------------------------------*/
 /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 HAL_Init();
//  SystemClock_ConfigS();
//
//
 	MX_GPIO_Init_Servo();
//
//  MX_TIM3_Init();
//  SERVO_Init();
 uint8_t pickup = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
 //uint8_t drop = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
 // Start PWM for motors
//  while (1)
//  {
	  if (pickup){
		  SystemClock_ConfigS();
		  MX_GPIO_Init_Servo();
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		  MX_TIM3_Init();
		  SERVO_Init();
			SERVO_MoveTo(0.0f);
			SERVO3_MoveTo(180.0f);
			HAL_Delay(1000);
			for (float angle = 0.0f; angle <= 100.0f; angle += 5.0f)
			{
				SERVO_MoveTo(angle);
				HAL_Delay(100);
			}
			SERVO2_MoveTo(0.0f);
			HAL_Delay(5000);
			for (float angle = 0.0f; angle <= 90.0f; angle += 5.0f)
			{
				SERVO2_MoveTo(angle);
				HAL_Delay(500);
			}
			for (float angle = 180.0f; angle >= 120.0f; angle -= 10.0f)
			{
				SERVO3_MoveTo(angle);
				HAL_Delay(300);
			}
			HAL_Delay(3000);
			SERVO3_MoveTo(180.0f);
			for (float angle = 90.0f; angle >= 0.0f; angle -= 5.0f)
			{
				SERVO2_MoveTo(angle);
				HAL_Delay(500);
			}
			SERVO2_MoveTo(0.0f);
			HAL_Delay(500);
			for (float angle = 100.0f; angle >= 0.0f; angle -= 5.0f)
			{
				SERVO_MoveTo(angle);
				HAL_Delay(100);
			}
			SERVO_MoveTo(0.0f);
			HAL_Delay(5000);
			//pickup = 0;
			//break;
			}
	    // Deinitialize PWM channels
//	    HAL_TIM_PWM_Stop(&htim3, SERVO_TIM_CHANNEL);
//	    HAL_TIM_PWM_Stop(&htim3, SERVO2_TIM_CHANNEL);
//
//	    // Deinitialize Timer
//	    HAL_TIM_PWM_DeInit(&htim3);
//
//	    // Deinitialize GPIOs
//	    HAL_GPIO_DeInit(SERVO_GPIO_PORT, SERVO_GPIO_PIN);
//	    HAL_GPIO_DeInit(SERVO2_GPIO_PORT, SERVO2_GPIO_PIN);
//
//	    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
//
//	    // Reset Peripheral Clocks
//	    __HAL_RCC_GPIOA_CLK_DISABLE();
//	    __HAL_RCC_TIM3_CLK_DISABLE();
	  else{
//
	  SystemClock_ConfigL();
	  MX_TIM2_Init();
	  MX_GPIO_Init_LFR();
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Motor1 PWM
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Motor2 PWM
//
//
while(1)
     // Read IR sensor inputs
	  { uint8_t ir_left = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); // Left IR sensor (PA0)
     uint8_t ir_right = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1); // Right IR sensor (PA1)
     uint8_t ir_middle = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3); // Middle IR sensor (PA3)
//
//
     if (ir_left == GPIO_PIN_SET && ir_right == GPIO_PIN_SET && ir_middle == GPIO_PIN_SET) // Both sensors detect the line //means black line  detected   //means IR led off
     {
         // Move forward
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // Motor1 forward
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // Motor2 forward
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, htim2.Init.Period * 0.8); // 80% duty cycle
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, htim2.Init.Period * 0.8); // 80% duty cycle
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, htim2.Init.Period * 0.20); // 80% duty cycle
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, htim2.Init.Period * 0.20); // 80% duty cycle
     }
     else if (ir_left == GPIO_PIN_SET && ir_right == GPIO_PIN_RESET && ir_middle == GPIO_PIN_SET) // Left sensor detects the line
     {
         // Turn left
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Motor1 stop
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // Motor2 forward
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10); // Motor1 off
//            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, htim2.Init.Period * 0.8); // Motor2 80% duty cycle
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, htim2.Init.Period * 0.22); // Motor2 80% duty cycle
     }
     else if (ir_left == GPIO_PIN_RESET && ir_right == GPIO_PIN_SET && ir_middle == GPIO_PIN_SET) // Right sensor detects the line
     {
         // Turn right
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // Motor1 forward
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Motor2 stop
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, htim2.Init.Period * 0.8); // Motor1 80% duty cycle
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, htim2.Init.Period * 0.22); // Motor1 80% duty cycle
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 10); // Motor2 off
     }
     else // No line detected
     {
         // Stop both motors
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Motor1 stop
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Motor2 stop
         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0); // Motor1 off
         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0); // Motor2 off
     }
     HAL_Delay(10);
	}}
//  /* USER CODE END 3 */
}
// Initialize the servo by configuring PWM limits
void SERVO_Init(void)
{
  // uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2; // Assuming APB1 prescaler = 2
  // uint32_t psc = htim2.Init.Prescaler + 1;
   uint32_t pwm_period = htim3.Init.Period + 1;
   // Calculate the pulse width ticks for the minimum and maximum angles
   min_pulse_ticks = (uint16_t)((SERVO_MIN_PULSE / 20.0f) * pwm_period);
   max_pulse_ticks = (uint16_t)((SERVO_MAX_PULSE / 20.0f) * pwm_period);
   // Start PWM on the configured channel
   HAL_TIM_PWM_Start(&htim3, SERVO_TIM_CHANNEL);
   HAL_TIM_PWM_Start(&htim3, SERVO2_TIM_CHANNEL);
   HAL_TIM_PWM_Start(&htim3, SERVO3_TIM_CHANNEL);
}
// Move the servo to a specific angle (0 to 180)
void SERVO_MoveTo(float angle)
{
   if (angle < 0.0f)
       angle = 0.0f;
   if (angle > 180.0f)
       angle = 180.0f;
   // Calculate pulse width corresponding to the angle
   uint16_t pulse = min_pulse_ticks + (uint16_t)((angle / 180.0f) * (max_pulse_ticks - min_pulse_ticks));
   // Set the pulse width for the PWM channel
   __HAL_TIM_SET_COMPARE(&htim3, SERVO_TIM_CHANNEL, pulse);
}
void SERVO2_MoveTo(float angle)
{
   if (angle < 0.0f)
       angle = 0.0f;
   if (angle > 180.0f)
       angle = 180.0f;
   // Calculate pulse width corresponding to the angle
   uint16_t pulse = min_pulse_ticks + (uint16_t)((angle / 180.0f) * (max_pulse_ticks - min_pulse_ticks));
   // Set the pulse width for the PWM channel
   __HAL_TIM_SET_COMPARE(&htim3, SERVO2_TIM_CHANNEL, pulse);
}
void SERVO3_MoveTo(float angle)
{
   if (angle < 0.0f)
       angle = 0.0f;
   if (angle > 180.0f)
       angle = 180.0f;
   // Calculate pulse width corresponding to the angle
   uint16_t pulse = min_pulse_ticks + (uint16_t)((angle / 180.0f) * (max_pulse_ticks - min_pulse_ticks));
   // Set the pulse width for the PWM channel
   __HAL_TIM_SET_COMPARE(&htim3, SERVO3_TIM_CHANNEL, pulse);
}
// Sweep the servo between 0 and 180 degrees
void SERVO_Sweep(void)
{
   for (float angle = 0.0f; angle <= 180.0f; angle += 1.0f)
   {
       SERVO_MoveTo(angle);
       HAL_Delay(10);
   }
   for (float angle = 180.0f; angle >= 0.0f; angle -= 1.0f)
   {
       SERVO_MoveTo(angle);
       HAL_Delay(10);
   }
}
void SystemClock_ConfigS(void)
{
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
   RCC_OscInitStruct.HSEState = RCC_HSE_ON;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
   HAL_RCC_OscConfig(&RCC_OscInitStruct);
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
   HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}
void SystemClock_ConfigL(void)
{
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	  {
	    Error_Handler();
	  }
}
/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
 /* USER CODE BEGIN TIM2_Init 0 */
 /* USER CODE END TIM2_Init 0 */
 TIM_ClockConfigTypeDef sClockSourceConfig = {0};
 TIM_MasterConfigTypeDef sMasterConfig = {0};
 TIM_OC_InitTypeDef sConfigOC = {0};
 /* USER CODE BEGIN TIM2_Init 1 */
 /* USER CODE END TIM2_Init 1 */
 htim2.Instance = TIM2;
 htim2.Init.Prescaler = 0;
 htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
 htim2.Init.Period = 65535;
 htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
 htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
 if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
 {
   Error_Handler();
 }
 sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
 if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
 {
   Error_Handler();
 }
 sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
 sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
 if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
 {
   Error_Handler();
 }
 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = 0;
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
 if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
 {
   Error_Handler();
 }
 if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN TIM2_Init 2 */
 /* USER CODE END TIM2_Init 2 */
 HAL_TIM_MspPostInit(&htim2);
}
/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{
   __HAL_RCC_TIM3_CLK_ENABLE();
   htim3.Instance = TIM3;
   htim3.Init.Prescaler = 71; // Prescaler for 1 MHz timer clock (72 MHz / 72)
   htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim3.Init.Period = 19999; // 20 ms period (1 MHz / 50 Hz)
   htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
   HAL_TIM_Base_Init(&htim3);
   TIM_OC_InitTypeDef sConfigOC = {0};
   sConfigOC.OCMode = TIM_OCMODE_PWM1;
   sConfigOC.Pulse = 0;
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, SERVO_TIM_CHANNEL);
   HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, SERVO2_TIM_CHANNEL);
   HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, SERVO3_TIM_CHANNEL);
}
static void MX_GPIO_Init_LFR(void)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */
 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOC_CLK_ENABLE();
 __HAL_RCC_GPIOD_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
 /*Configure GPIO pins : PA0 PA1 */
 GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
 /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
 GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
static void MX_GPIO_Init_Servo(void)
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   GPIO_InitStruct.Pin = SERVO_GPIO_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(SERVO_GPIO_PORT, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = SERVO2_GPIO_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(SERVO2_GPIO_PORT, &GPIO_InitStruct);
   GPIO_InitStruct.Pin = SERVO3_GPIO_PIN;
   GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(SERVO3_GPIO_PORT, &GPIO_InitStruct);

   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
   GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Set as push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
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

