/* USER CODE BEGIN Header */
/** FILE          : main.c
 PROJECT       : Debit Machine Advanced
 PROGRAMMER    : Tej Partap Singh Sidhu
 DATE CREATED :  Aug 17, 2021
 DESCRIPTION   : Demonstrates a debit machine banking transaction that implements a state machine.
 	 	 	 	 In addition to basic functionalities, an RGB led and speaker is
 	 	 	 	 also connected to show appropriate outputs as per transaction
 	 	 	 	 status.


 Switches are assigned as follows
 note: these pins are set in the debounceInit functions and do not need to be configured in cube
 PA0			PA1			PA4			PA3
 chequing		savings		ok			cancel

 In addition to these, pin PA5 and PA6 are connected for output to led
 PA7 is connected for PWM output to connect to speaker.
 Note:
 1. Don't use PA2 as it is connected to VCP TX and you'll
 lose printf output ability.
 2. Bank balance and correct pin can be changed by changing corresponding macro value(#define).
    Bank makes its decision as OK or not based on these values.

 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <unistd.h>
#include <stdio.h>
#include "debounce.h"
#include "HD44780.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VALID_TONE (uint8_t)1   // Tone to play when transaction successful
#define INVALID_TONE (uint8_t)0  // Tone to play when transaction not successful
#define BANK_RESPONSE_VALID_TRANSACTION "OK"
#define BANK_RESPONSE_INVALID_TRANSACTION "NO"
#define BANK_BALANCE 10000   // Bank balance in dollar
#define CORRECT_PIN 3621    // Correct ATM pin saved in bank for user
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const int16_t chequingPbPin = 0; //setting the pin assigned to each pb
static const int16_t savingsPbPin = 1;		//don't use pin 2 as it's connected
static const int16_t okPbPin = 4;		//to VCP TX
static const int16_t cancelPbPin = 3;

enum pushButton {
	none, chequing, savings, ok, cancel
};
//enumerated values for use with if
//(pbPressed == value) type conditional
//statements

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void displayChequingOrSaving(void);
void displayTransactionCancelled(void);
enum pushButton checkSavingsOrChequing(void);
void displayEnterAmount(void);
void displayEnterPin(void);
void displayInvalidAmount(void);
void displayPin(void);
void displayInvalid(void);
void displayCheckingWithBank(void);
void displaySuccessful(void);
unsigned int checkIfPinRecd();
uint8_t checkWithBank(float amount, unsigned int pin, float bankBalance);
void displayRemainingBalance(float bankBalance);
void playSpeaker(uint8_t tone);
void turnLedOn(uint8_t PIN);
void turnLedOff(uint8_t PIN);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// FUNCTION      : setTone
// DESCRIPTION   : Calculates the PWM Period needed to obtain the freq
//				 : passed and the duty cycle of the PAM to
//				 : 50% (1/2 of the period)
// PARAMETERS    : int32 freq - frequency of the output
// RETURNS       : nothing
void setTone(int32_t freq) {
	int32_t pwmPeriod = 1000000000 / (freq * 250); //value can vary between 2 and 65535
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = pwmPeriod;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
		!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pwmPeriod / 2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	/* adding this as ST Tech Support said PWM should be stopped before
	 * calling HAL_TIM_PWM_ConfigChannel and I've been getting flakey start-up
	 * i.e.: sometime PWM starts up, other times the line remains stuck high.
	 **************************************/
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	/*************************************/
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
		!= HAL_OK) {
		Error_Handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
		!= HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim1);
}

// FUNCTION      : waitForPBRelease
// DESCRIPTION   : Loops until the PB that is currently
//				 : pressed and at a logic low
//				 : is released. Release is debounced
// PARAMETERS    : pin - pin number
//                 port- port letter ie 'A'
// RETURNS       : nothing
void waitForPBRelease(const int16_t pin, const char port) {
	while (deBounceReadPin(pin, port, 10) == 0) {
		//do nothing wait for key press to be released
	}
}

// FUNCTION      : startUpLCDSplashScreen()
// DESCRIPTION   : displays Debit Demo for 2s
//                 on line 1 of the display and
//				 : Disappears
// PARAMETERS    : None
// RETURNS       : nothing
void startUpLCDSplashScreen(void) {
	char stringBuffer[16] = { 0 };
	HD44780_GotoXY(0, 0);
	snprintf(stringBuffer, 16, "   Debit Demo");
	HD44780_PutStr(stringBuffer);
	HAL_Delay(2000);
	HD44780_ClrScr();
}

// FUNCTION      : pulsePWM
// DESCRIPTION   : Turns on the PWM for the pulseTime in ms
//                 provided and then turns off PWM
// PARAMETERS    : address of Timer Handle var (e.g.: &htim1)
//                 pulseTime in ms
// RETURNS       : nothing
void pulsePWM(TIM_HandleTypeDef* htim1, int32_t pulseTime) {
	HAL_TIMEx_PWMN_Start(htim1, TIM_CHANNEL_1);
	HAL_Delay(pulseTime);
	HAL_TIMEx_PWMN_Stop(htim1, TIM_CHANNEL_1);
}

//  FUNCTION      : pushButtonInit
//   DESCRIPTION   : Calls deBounceInit to initialize ports that
//                   will have pushbutton on them to be inputs.
//			         Initializing PA0,PA1,PA4 and PA3
//                   Switches are assigned as follows
//                   PA0			PA1			PA4			PA3
//                   chequing		savings		ok			cancel
//
//                   Note: Don't use PA2 as it is connected to VCP TX and you'll
//                   lose printf output ability.
//   PARAMETERS    : None
//   RETURNS       : nothing
void pushButtonInit(void) {
	deBounceInit(chequingPbPin, 'A', 1); 		//1 = pullup resistor enabled
	deBounceInit(savingsPbPin, 'A', 1); 		//1 = pullup resistor enabled
	deBounceInit(okPbPin, 'A', 1); 			//1 = pullup resistor enabled
	deBounceInit(cancelPbPin, 'A', 1); 		//1 = pullup resistor enabled
}

// FUNCTION      : displayWelcome()
// DESCRIPTION   : clears the LCD display and displays
//                 Welcome on line 1 of the display
// PARAMETERS    : None
// RETURNS       : nothing
void displayWelcome(void) {
	char stringBuffer[16] = { 0 };
	HD44780_ClrScr();
	snprintf(stringBuffer, 16, "Welcome ");
	HD44780_PutStr(stringBuffer);
	HAL_Delay(2000);
}

// FUNCTION      : displayAmount()
// DESCRIPTION   : clears the LCD display and displays
//                 the $amount received on line 1 of the display
// PARAMETERS    : float - amount to display
// RETURNS       : nothing
void displayAmount(float amount) {
	char stringBuffer[16] = { 0 };
	HD44780_ClrScr();
	HD44780_GotoXY(0, 0);
	snprintf(stringBuffer, 16, "$%.2f", amount);
	HD44780_PutStr(stringBuffer);
}

// FUNCTION      : checkIfAmountRecd()
// DESCRIPTION   :
// PARAMETERS    : none
// RETURNS       : float, the amount in $ to be debited
float checkIfAmountRecd() {
	float debitAmount = 0;
	printf("Waiting for debitAmount to be received on serial port\r\n");
	int16_t result = 0;
	result = scanf("%f", &debitAmount);
	if (result == 0)		//then somehow non-float chars were entered
	{						//and nothing was assigned to %f
		fpurge(STDIN_FILENO); //clear the last erroneous char(s) from the input stream
	}
	return debitAmount;
}

// FUNCTION      : checkOkOrCancel()
// DESCRIPTION   : Checks whether the OK or Cancel
//                 button has been pressed.
// PARAMETERS    : none
// RETURNS       : int8_t, 3 if cancel pressed, 4 if ok
//                 ok pressed. 0 returned if neither
//                 has pressed.
enum pushButton checkOkOrCancel(void) {
	if (deBounceReadPin(cancelPbPin, 'A', 10) == 0) {
		//then the cancel pushbutton has been pressed
		return cancel;
	}
	else if (deBounceReadPin(okPbPin, 'A', 10) == 0) {
		//then ok pressed
		return ok;
	}
	return none; //as ok or cancel was not pressed.
}

// FUNCTION      : displayOkOrCancel()
// DESCRIPTION   : displays "OK or Cancel?" on line 2 of LCD
// PARAMETERS    : none
// RETURNS       : nothing.
void displayOkCancel(void) {
	printf("Waiting for ok or cancel\r\n");
	char stringBuffer[16] = { 0 };
	HD44780_GotoXY(0, 1); //move to second line first position
	snprintf(stringBuffer, 16, "OK or Cancel?");
	HD44780_PutStr(stringBuffer);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	printf("Debit Card State Machine\r\n");
	HD44780_Init();
	/* setup Port A bits 0,1,2 and 3, i.e.: PA0-PA3 for input */
	pushButtonInit();
	startUpLCDSplashScreen();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		static float amount = 0;           //used to hold the transaction amount
		static int8_t transactionState = 1;
		static int8_t lastState = 1; // captures last state from where the transaction proceeded
		static int16_t transactionNumber = 1; // Keeps track of number of transaction made
		enum pushButton pbPressed = none; //will hold pushbutton defined above depending on
										  //the pushbutton pressed
		static unsigned int pin = 0;    // Debit pin to be passed by user
		static int bankBalance = BANK_BALANCE;  // Bank balance of user

		/*states:   1   display Welcome Screen, wait for $ amount input from Serial port
		 2   @ amount Received, waiting for Ok or Cancel button
		 3   OK received, waiting for chequing or Savings button
		 4   C or S received, waiting for PIN to be entered from Serial Port
		 5   Pin Correct, send transaction data to bank. Waiting
		 for OK back from Bank If OK from Bank received. Print
		 Receipt, Record transaction. Move back to State 1.
		 6   Cancel Pressed. Display "Transaction Cancelled" back to state 1
		 */

		switch (transactionState) {

			/********  State 1: Check if amount has been received ********/
		case 1:
			printf("\n*********** Transaction %d ***********\r\n",
				transactionNumber);
			displayWelcome();
			displayEnterAmount();
			amount = checkIfAmountRecd();
			if (amount != 0)        //returns a 0 if an transaction amount has
			{ 					//NOT been received on the serial port.
				displayAmount(amount); //but if we're we've received a debitAmount
				displayOkCancel();	//so display it and the prompt ok or cancel
				transactionState = 2;//and do that before we move on to state 2
				lastState = 1;
			}
			else {
				displayInvalidAmount();
			}
			transactionNumber++;
			break;

			/********** State 2: Amount/Pin received, waiting for Ok or Cancel **********/
		case 2:

			pbPressed = checkOkOrCancel();
			if (pbPressed != none) {
				if (pbPressed == cancel) {
					//then cancel was pressed.
					transactionState = 6;
				}
				else if (pbPressed == ok) {
					//then ok pressed
					printf("OK Pressed\r\n");
					if (lastState == 1) {  // has come here from state 1
						displayChequingOrSaving();
						transactionState = 3;
					}
					else if (lastState == 4) {  // has come here from state 4
						transactionState = 5;
					}
				}
			}
			break;

			/******** State 3: Check for savings or chequing **********/
		case 3:
			pbPressed = checkSavingsOrChequing();

			if (pbPressed != none) {
				if (pbPressed == cancel) {
					//then cancel was pressed.
					transactionState = 6;
				}
				else if (pbPressed == chequing) {
					//then chequing pressed
					printf("C %.2f\r\n", amount);
					transactionState = 4;
					displayEnterPin();
				}
				else if (pbPressed == savings) {
					//then savings pressed
					printf("S %.2f\r\n", amount);
					transactionState = 4;
					displayEnterPin();
				}
			}
			break;
			/*********** State 4: Check if pin has been received ***************/
		case 4:
			pin = checkIfPinRecd();
			if (pin != 0) {        //returns a 0 if a pin has
								   //NOT been received on the serial port.
				displayPin();      // but if we've received a pin
				displayOkCancel();	// display **** and display Ok or Cancel
				transactionState = 2;//and do that before we move on to state 2
				lastState = 4;
			}
			else {
				displayInvalid();
				transactionState = 6;
			}
			break;

			/*********** State 5: Bank verification stage *************/
		case 5:
			displayCheckingWithBank();
			uint8_t bankResponse = 0;
			bankResponse = checkWithBank(amount, pin, bankBalance); // returns 1 if bank says ok.

			if (bankResponse == 1) { // pin is correct
				printf("Transaction Successful\r\n\n");
				bankBalance -= amount;
				displaySuccessful();
				turnLedOn(VALID_LED_Pin);
				playSpeaker(VALID_TONE);
				transactionState = 1; // Go to state 1 for another transaction
				turnLedOff(VALID_LED_Pin);
				HD44780_ClrScr();  // Used here to avoid blocking
								   // due to displaySuccessful function
			}
			else {   // Either balance low or pin incorrect
				printf("Invalid Pin or low balance!\r\n");
				displayInvalid();
				turnLedOn(INVALID_LED_Pin);
				playSpeaker(INVALID_TONE);
				transactionState = 6; // Go to transaction cancellation
				turnLedOff(INVALID_LED_Pin);
				HD44780_ClrScr();  // Used here to avoid blocking
								   // due to displayInvalidPin function
			}
			displayRemainingBalance(bankBalance);
			break;

			/**********************  State 6: Transaction Cancellation ****************/
		case 6:
			printf("Transaction Cancelled\r\n\n");
			displayTransactionCancelled();
			transactionState = 1;
			break;
		default:
			break;
		} //closing brace for switch

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, VALID_LED_Pin|INVALID_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : VALID_LED_Pin INVALID_LED_Pin */
  GPIO_InitStruct.Pin = VALID_LED_Pin|INVALID_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// FUNCTION      : displayChequingOrSaving
// DESCRIPTION   : displays "Chequing or Savings?" on  LCD
// PARAMETERS    : none
// RETURNS       : nothing.
void displayChequingOrSaving(void) {
	printf("Waiting for chequing or savings\r\n");
	char stringBuffer1[16] = { 0 };
	char stringBuffer2[16] = { 0 };
	HD44780_ClrScr();

	HD44780_GotoXY(0, 0); //move to first line first position
	snprintf(stringBuffer1, 16, "Chequing or");
	HD44780_PutStr(stringBuffer1);

	HD44780_GotoXY(0, 1);
	snprintf(stringBuffer2, 16, "Savings?");
	HD44780_PutStr(stringBuffer2);
}

// FUNCTION      : displayTransactionCancelled
// DESCRIPTION   : displays "TransactionCancelled" on LCD
// PARAMETERS    : none
// RETURNS       : nothing.
void displayTransactionCancelled(void) {
	char stringBuffer1[16] = { 0 };
	char stringBuffer2[16] = { 0 };
	HD44780_ClrScr();

	HD44780_GotoXY(0, 0); //move to first line first position
	snprintf(stringBuffer1, 16, "Transaction");
	HD44780_PutStr(stringBuffer1);

	HD44780_GotoXY(0, 1);
	snprintf(stringBuffer2, 16, "Cancelled");
	HD44780_PutStr(stringBuffer2);
	HAL_Delay(3000);
	HD44780_ClrScr();
}

// FUNCTION      : checkSavingsOrChequingl()
// DESCRIPTION   : Checks whether the user is using a chequing
//				   or savings account.
// PARAMETERS    : none
// RETURNS       : int8_t pushButton: 1 if chequing pressed
//				  		              2 if savings pressed.
//                          		  0 returned if neither pressed
//
enum pushButton checkSavingsOrChequing(void) {
	if (deBounceReadPin(chequingPbPin, 'A', 10) == 0) {
		//then the chequing pushbutton has been pressed
		return chequing;
	}
	else if (deBounceReadPin(savingsPbPin, 'A', 10) == 0) {
		//then savings pressed
		return savings;
	}
	else if (deBounceReadPin(cancelPbPin, 'A', 10) == 0) {
		//then cancel pressed
		return cancel;
	}
	return none; //nothing or ok was pressed
}

// FUNCTION      : displayEnterAmount()
// DESCRIPTION   : displays "Enter Amount" on LCD
// PARAMETERS    : none
// RETURNS       : nothing.
void displayEnterAmount(void) {
	char stringBuffer[16] = { 0 };
	HD44780_ClrScr();

	HD44780_GotoXY(0, 0); //move to first line first position
	snprintf(stringBuffer, 16, "Enter Amount ");
	HD44780_PutStr(stringBuffer);

}

// FUNCTION      : displayEnterPin
// DESCRIPTION   : displays "Enter Pin" on LCD
// PARAMETERS    : none
// RETURNS       : nothing.
void displayEnterPin(void) {
	char stringBuffer[16] = { 0 };
	HD44780_ClrScr();

	HD44780_GotoXY(0, 0); //move to first line first position
	snprintf(stringBuffer, 16, "Enter Pin ");
	HD44780_PutStr(stringBuffer);
}

// FUNCTION      : displayInvalidAmount
// DESCRIPTION   : displays "Invalid Amount" on LCD
// PARAMETERS    : none
// RETURNS       : nothing.
void displayInvalidAmount(void) {
	char stringBuffer1[16] = { 0 };
	char stringBuffer2[16] = { 0 };
	HD44780_ClrScr();

	HD44780_GotoXY(0, 0); //move to first line first position
	snprintf(stringBuffer1, 16, "Invalid Amount");
	HD44780_PutStr(stringBuffer1);
	HD44780_GotoXY(0, 1);
	snprintf(stringBuffer2, 16, "Try Again");
	HD44780_PutStr(stringBuffer2);
	HAL_Delay(2000);
	HD44780_ClrScr();
}

// FUNCTION      : displayPin
// DESCRIPTION   : displays "****" on LCD
// PARAMETERS    : none
// RETURNS       : nothing.
void displayPin(void) {
	HD44780_ClrScr();
	char stringBuffer[16] = { 0 };
	HD44780_GotoXY(0, 0); //move to second line first position
	snprintf(stringBuffer, 16, "****");
	HD44780_PutStr(stringBuffer);

}

// FUNCTION      : displayInvalid
// DESCRIPTION   : displays "Invalid Pin" on LCD
// PARAMETERS    : none
// RETURNS       : nothing.
void displayInvalid(void) {
	char stringBuffer1[16] = { 0 };
	char stringBuffer2[16] = { 0 };
	HD44780_ClrScr();

	HD44780_GotoXY(0, 0); //move to first line first position
	snprintf(stringBuffer1, 16, "Invalid Pin Or");
	HD44780_PutStr(stringBuffer1);
	HD44780_GotoXY(0, 1); //move to second line first position
	snprintf(stringBuffer2, 16, "Low balance");
	HD44780_PutStr(stringBuffer2);
}

// FUNCTION      : displayCheckingWithBank
// DESCRIPTION   : displays "Checking With Bank" on LCD
// PARAMETERS    : none
// RETURNS       : nothing.
void displayCheckingWithBank(void) {
	char stringBuffer1[16] = { 0 };
	char stringBuffer2[16] = { 0 };
	HD44780_ClrScr();

	HD44780_GotoXY(0, 0); //move to first line first position
	snprintf(stringBuffer1, 16, "Checking With");
	HD44780_PutStr(stringBuffer1);
	HD44780_GotoXY(0, 1); //move to second line first position
	snprintf(stringBuffer2, 16, "Bank...");
	HD44780_PutStr(stringBuffer2);
	HAL_Delay(2000);

}
// FUNCTION      : displaySuccessful
// DESCRIPTION   : displays "Successful" on LCD
// PARAMETERS    : none
// RETURNS       : nothing.
void displaySuccessful(void) {
	char stringBuffer1[16] = { 0 };
	char stringBuffer2[16] = { 0 };
	HD44780_ClrScr();
	HAL_Delay(10);
	HD44780_GotoXY(0, 0); //move to first line first position
	snprintf(stringBuffer1, 16, "Transaction");
	HD44780_PutStr(stringBuffer1);
	HD44780_GotoXY(0, 1); //move to second line first position
	snprintf(stringBuffer2, 16, "Successful");
	HD44780_PutStr(stringBuffer2);
	HAL_Delay(10);
}

// FUNCTION      : checkIfAmountRecd
// DESCRIPTION   : Checks if the pin has been received as an integer
// PARAMETERS    : none
// RETURNS       : unsigned int: pin passed by the user
unsigned int checkIfPinRecd(void) {
	unsigned int pin = 0;
	printf("Waiting for pin to be received on serial port\r\n");
	int16_t result = 0;
	result = scanf("%d", &pin);
	if (result == 0)		//then somehow non-integer chars were entered
	{						//and nothing was assigned to %d
		fpurge(STDIN_FILENO); //clear the last erroneous char(s) from the input stream
	}
	return pin;
}

// FUNCTION      : checkWithBank(float amount, unsigned int pin, float bankBalance)
// DESCRIPTION   : Checks from the bank(Serial port) if pin is ok and balance is sufficient
// PARAMETERS    : float amount: transaction amount requested by the user
//				   unsigned int pin: pin passed by the user
//				   float bankBalance: bank balance of user

// RETURNS       : uint8_t , 0 : if bank says not ok
//                           1 : if bank says ok
uint8_t checkWithBank(float amount, unsigned int pin, float bankBalance) {
	printf("Checking with bank...\r\n");

	if ((amount <= bankBalance && amount > 0) && (pin == CORRECT_PIN)) {
		printf("Bank response : OK\r\n");
		return 1;
	}
	else {
		printf("Bank response : Not OK\r\n");
		return 0;
	}
}

// FUNCTION      : displayRemainingBalance
// DESCRIPTION   : prints remaining balance on the serial port
// PARAMETERS    : float bankBalance: bank Balance of user

// RETURNS       : void
void displayRemainingBalance(float bankBalance) {
	printf("Available Balance : %.2f\r\n", bankBalance);
	return;
}

/**
 FUNCTION      : playSpeaker
 DESCRIPTION   :
 This function plays the speaker based on the
 tone passed as argument
 PARAMETERS    :
 uint8_t tone: Tone to play on speaker
			   1 for valid, 0 for invalid

 RETURNS       :
 void
 */
void playSpeaker(uint8_t tone) {
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	if (tone == VALID_TONE) {
		uint16_t freq = 150;
		setTone(freq);
		pulsePWM(&htim1, 1500);

	}
	else {
		uint16_t freq = 220;
		for (int i = 0; i < 300; i++) {
			setTone(freq);
			pulsePWM(&htim1, 10);
			freq += 10;
			if (freq > 600)
				freq = 220;
		}
	}
}

/**
 FUNCTION      : turnLedOn
 DESCRIPTION   :
 This function sets the pin passed as argument to "on".
 PARAMETERS    :
 uint8_t pin: Pin  to turn on

 RETURNS       :
 void
 */
void turnLedOn(uint8_t pin) {
	HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_SET);
	return;
}

/**
 FUNCTION      : turnLedOff
 DESCRIPTION   :
 This function sets the pin passed as argument to "off".
 PARAMETERS    :
 uint8_t pin : Pin to turn off

 RETURNS       :
 void
 */
void turnLedOff(uint8_t pin) {
	HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_RESET);
	return;
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
