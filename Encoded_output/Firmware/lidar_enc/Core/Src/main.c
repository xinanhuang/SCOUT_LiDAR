/* USER CODE BEGIN Header */
/**
 *
 *
 *
 */

/****** TO-DO ******/

// 1. implement a lookup table for sin and cos
// 2. use DMA for quadture encoder
// replace sprintf with my own version of int to BCD converter




/****** PINS LIST *******/

// PA2  - TX pin, unused but initalized, possible for sending commands to adjust lidar setting
// PA3  - RX pin, for reciving SF30/C lidar data

// PA11/12 - USB virtual com port

// PA0  - interrupt for quadrature index signal
// PB6  - Channel B of Quadrature encoder
// PB7  - Channel A of Quadrature encoder

// PB0-PB2, PB10 - stepper motor control (in the order of A1, A2, B1, B2)
// PB11 - stepper motor homing interrupt


//PB 3, 14, 15 - swd





/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
# define PI 3.1415926f
//cosine and sine table


/* encoder counter */
// timer4, channel 1,2//
// external interrupt for index reset//
void init_encoder(void);
void EXTI0_IRQHandler(void);



void stepper_init(void);

// winding activations
// Forward:   A2B1 -> A2B2 -> A1B2 -> A1B1
// Backward:  A2B2 -> A2B1 -> A1B1 -> A1B2
void A2B1(void);
void A2B2(void);
void A1B2(void);
void A1B1(void);
void full_step_fw(void);
void full_step_rv(void);

volatile int stepper_condi = 0;
volatile int step_count = 0;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void get_dec_str (uint8_t* str, int val1, int val2)
{
  uint8_t i;
  for(i=8; i<=12; i++)
  {
    str[12-i] = (uint8_t) ((val1 % 10UL) + '0');
    val1/=10;
  }

  str[5 ] = ' ';

  for(i=2; i<=6; i++)
  {
    str[12-i] = (uint8_t) ((val2 % 10UL) + '0');
    val2/=10;
  }

  str[11] = '\n';
  str[12] = '\0';
}



/// encode xyz data into packets
void data_encode(int16_t x, int16_t y, int16_t z, char *pkt)
{
	// x
	pkt[0] = 0x00 | (x >> 12 & 0xF);
	pkt[1] = 0x10 | (x >> 8  & 0xF);
	pkt[2] = 0x20 | (x >> 4  & 0xF);
	pkt[3] = 0x30 | (x       & 0xF);
	//y
	pkt[4] = 0x40 | (y >> 12 & 0xF);
	pkt[5] = 0x50 | (y >> 8  & 0xF);
	pkt[6] = 0x60 | (y >> 4  & 0xF);
	pkt[7] = 0x70 | (y       & 0xF);
	//z
	pkt[8]  = 0x80 | (z >> 12 & 0xF);
	pkt[9]  = 0x90 | (z >> 8  & 0xF);
	pkt[10] = 0xA0 | (z >> 4  & 0xF);
	pkt[11] = 0xB0 | (z       & 0xF);
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
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

   uint8_t  LIDAR_RX_BUFFER[2];
   int byteState = 0;
   int byteH = 0;

   uint8_t  send_pkt[12];




	int  distance   =0;


	int  rotary_raw = 0;

	float ang_step = 0.0f; // value from stepper motor
	float sin_step = 0.0f;
	float cos_step = 0.0f;

	float ang_enc = 0.0f;  // value from encoder
	float sin_enc = 0.0f;
	float cos_enc = 0.0f;

	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;








	uint8_t Rbyte=0;
	uint8_t Hbyte=0;
  // HAL_UART_Transmit(&huart2,(uint8_t *)"#R2:",3,10);
   //HAL_UART_Transmit(&huart2,(uint8_t *)"#U2:",3,10);
   //HAL_UART_Transmit(&huart2,(uint8_t *)"#S1:",3,10);
   //HAL_UART_Transmit(&huart2,(uint8_t *)"#Z0:",3,10);
   init_encoder();
   stepper_init();



   /* motor stuff*/
   /* try to use integer only */

   int mode_select = 1; //  1-scanning mode, 0-ir motor pid mode


   float kp = 0.0f;
   float ki = 0.0f;
   float kd = 0.0f;

   int desired_enc = 0;
   int err         = 0;

   /* calibrate stepper motor position ( go to homing location)*/
   // more precis callibration happens per full step instead



   while (1)
   {


	   while (!(GPIOA->IDR &= 0x10))
	   {
		   full_step_rv();
	   }

	   while (step_count < 350)
	   {
			if (stepper_condi == 0)
			{
				A2B1();
				stepper_condi ++;
			}

			else if (stepper_condi == 1)
			{
				A2B2();
				stepper_condi ++;
			}

			else if (stepper_condi == 2)
			{
				A1B2();
				stepper_condi ++;
			}

			else if (stepper_condi == 3)
			{
				A1B1();
				stepper_condi = 0;
			}

			step_count ++;
			HAL_Delay(1);
	   }



	  /* determining if goes to motor control loop for IR or lidar scanning */



	  ///////////////////
	  /* motor control */
	  ///////////////////
	   while (mode_select == 0)
	   {

		  // * planar stage * //
		  // read encoder value and find the error
		   rotary_raw =TIM4->CNT;
		   err = desired_enc - rotary_raw;

		  /*pid stuff*/

		  // set voltage by changing pwm


		  // * vertical stage *//
		  // no pid since stepper needs to step one at a time, gain only is good enough

		  // increase the stepper motor speed? (dunno how to do that without breaking the loop, like reduce the time between steps?

	   }
	  /* USER CODE END 2 */



	  /* USER CODE BEGIN WHILE */




	   ///////////////////
	   /* scanning loop */
	   ///////////////////
	   if( step_count <1025 && mode_select == 1 )
	   {
		   HAL_UART_Transmit(&huart2,(uint8_t *)"#Y:",3,10);  // start laser firing

	   }

	   while (step_count <1025 && mode_select == 1)
	   {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		  /* send to usart 2 and manage the lidar setting*/

		  /* begin lidar transmission*, use exception loop and a flat to start */

		 ////////////////
		 // UART stuff //
		 ////////////////
		  HAL_UART_Receive_DMA(&huart2, LIDAR_RX_BUFFER, 2);
		  rotary_raw =TIM4->CNT;
			// Parse the distance bytes.
			for (int i = 0; i < 2; ++i) {



				if (LIDAR_RX_BUFFER[i] & 0x80) {
					byteState = 1;
					byteH = LIDAR_RX_BUFFER[i] & 0x7F;

				}

				else {
					if (byteState) {

						byteState = 0;
						int distance = (byteH << 7) | LIDAR_RX_BUFFER[i];

						if (distance < 16000 && distance >0)
						{
							  // instead of sprintf we are rollign with our own bcd conversion to char

							//sprintf(send, "%d\n", distance);
							// calculation (use float, might cause issues later on with 10khz)

							ang_step = ((float)(step_count-512)*PI)/1024.0f;
							sin_step = sinf(ang_step);
							cos_step = cosf(ang_step);


							//cos_step = cosf(ang_step);

							ang_enc = ((float)rotary_raw*PI)/1800.0f;
							sin_enc = sinf(ang_enc);
							cos_enc = cosf(ang_enc);


							//cos_enc = cosf(ang_enc);

							x = cos_step*sin_enc*(float)distance;
							y = cos_step*cos_enc*(float)distance;
							z = sin_step*(float)distance;


							/* data packing */
							// to simplify the process we each 16bit variable into 4 bytes of data


							data_encode((int16_t) x, (int16_t) y, (int16_t) z, send_pkt);
							CDC_Transmit_FS(send_pkt, 12);





						}
					}
				}
			}

			if (step_count  == 1024)
			{
				send_pkt[0] = 0xC0;
				HAL_UART_Transmit(&huart2,(uint8_t *)"#N:",3,10);  // stop laser firing
				CDC_Transmit_FS(send_pkt, 1);  // end scan
			}


	   }

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1440000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */













/*** encoder stuff ***/
void init_encoder(void)
{
	/* timer 4 channel 1 and channel 2 (PB6, PB7) for encoder */
	RCC->AHB1ENR |= 0x2;  // enable port B
	RCC->APB1ENR |= 0x4;  // enable timer 4

	GPIOB->MODER  |= 0xA000;      // set to alternate function for pin6 amd 7
	GPIOB->PUPDR  |= 0x5000;      // set to pull up resistor for pin 6 and 7
	GPIOB->AFR[0] |= 0x22000000;  // set AFR to tim2-tim5

	TIM4->ARR = 0xFFFF;       // set max value

	TIM4->CCMR1  |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	TIM4->SMCR   |= 0x3;  // set to count both rising and fallign edge of AB channels, with reset mode enabled
	TIM4->CR1    |= 0x1;  // enable counter


	/* external interrupt, call encoder_reset() on rising edges */
	// PA0 //
	__disable_irq();         // disable iterupt before congfiuring
	RCC->AHB1ENR  |= 0x1;    // enable port A
	RCC->APB2ENR  |= 0x4000; // enable systemconfig clock
    GPIOA->PUPDR  |= 0x1;    // pull up for pin A0

	SYSCFG->EXTICR[0] |= 0x0; // EXIT0, Port A
	EXTI->IMR         |= 0x1; // mask exti 0
	EXTI->RTSR        |= 0x1; // set to rising edge
	NVIC_EnableIRQ(EXTI0_IRQn); // reenable exti0

	__enable_irq();          // reanble interupt
}


void EXTI0_IRQHandler(void)
{

  // can do away the if statements since we will be rotating in one direction only
	if(TIM4->CR1 & 0x10)
	{
	 TIM4->CNT     = 1800;
	}
	else
	{
	 TIM4->CNT     = 0;
	}

	/* increase stepper motor step */

	if (stepper_condi == 0)
	{
		A2B1();
		stepper_condi ++;
	}

	else if (stepper_condi == 1)
	{
		A2B2();
		stepper_condi ++;
	}

	else if (stepper_condi == 2)
	{
		A1B2();
		stepper_condi ++;
	}

	else if (stepper_condi == 3)
	{
		A1B1();
		stepper_condi = 0;
	}


	step_count ++;
	/* check end condition */
	EXTI->PR = 0x1; //clear interrupt
}



void stepper_init(void)
{
	// stepper
	RCC->AHB1ENR   |= 0x2;
	GPIOB->MODER   |= 0x100015;

	//stepper interrupt
	RCC->AHB1ENR   |= 0x1;
	GPIOA->PUPDR   |= 0x200;


}
// winding activations
void A2B1(void)
{
	GPIOB->BSRR |= 0x4010006;
}

void A2B2(void)
{
	GPIOB->BSRR |= 0x50402;
}

void A1B2(void)
{
	GPIOB->BSRR |= 0x60401;
}

void A1B1(void)
{
	GPIOB->BSRR |= 0x4020005;
}



void full_step_fw(void)
{
	A2B1();
	HAL_Delay(1);
	A2B2();
	HAL_Delay(1);
	A1B2();
	HAL_Delay(1);
	A1B1();
	HAL_Delay(1);

}


void full_step_rv(void)
{
	A2B2();
	HAL_Delay(1);
	A2B1();
	HAL_Delay(1);
	A1B1();
	HAL_Delay(1);
	A1B2();
	HAL_Delay(1);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
