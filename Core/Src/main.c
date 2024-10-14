/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FDCAN.h"
#include "CAN_MotherBoard.h"
#include "Ack.h"

#include "machineSettings.h"
#include "MachineErrors.h"
#include "MotorComms.h"
#include "SysObserver.h"

#include "userButtons.h"
#include "TowerLamp.h"
#include "SMPS.h"
#include "MachineSensors.h"
#include "mcp23017.h"
#include "Eeprom.h"
#include "Log.h"

#include "Struct.h"
#include "StateMachine.h"
#include "CommonConstants.h"

#include "BT_Fns.h"
#include "BT_Machine.h"
#include "BT_Console.h"

#include "MB_LEDs.h"
#include "AC_SSR.h"
#include "TD_Pot.h"
#include "DataRequest.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MCP23017_ADDRESS	0x21
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

DAC_HandleTypeDef hdac1;

FDCAN_HandleTypeDef hfdcan1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
machineSettingsTypeDef msp;
userSettings u;
internalSettings I;
CardingMc C;


machineSettingsTypeDef ps; // piecing settings
machineSettingsTypeDef msp_BT;
machineParamsTypeDef mcParams;
MBErrorsTypeDef MBE;
MachineErrorsTypedef ME;

SetupMotor SU[8];
ACKTypeDef ack;

StateTypeDef S;
DiagnosticsTypeDef D;
RunTime_TypeDef R[8];
ExtendedRunTime_TypeDef ER[2];//extended data frame from cylinder inverters

SysObserver SO;
BTRecvMsg_TypeDef BT;
BTConsole BTCmd;

userBtns usrBtns;
MCP23017_HandleTypeDef hmcp;
MCP23017_PortB mcp_portB;
MCP23017_PortB mcp_portB_whichSensor;
MCP23017_PortB mcp_portB_sensorVal;
MCP23017_PortA mcp_portA; // for AC SSR
SensorTypeDef sensor;
Log L;
TDP tdp;

DataReq DR;


char BufferRec[150];
char BufferTransmit[150] ;// Buffer for Transmit
char LogBuffer[2048];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t stopSMPS = 0;
uint8_t timer7Count = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  UNUSED(htim);
  if(htim==&htim17){	// ACK TIMER - 15ms
	ack.waitingForAckResult = 0;
	if (ack.motorAcksCheck == ack.motorAcksRecvd){
		ack.ackResult = ACK_SUCCESS;
	}else{
		ack.ackResult = ACK_FAIL;
		if (ack.ackChkType == CRITICAL_ACK){
			ME.ErrorFlag = 1;
			ME_addErrors(&ME,ERR_SYSTEM_LEVEL_SOURCE, SYS_ACK_ERROR, ERROR_SOURCE_SYSTEM, ack.ackForWhat); // maybe later find out which ACK failed.
			S.SMPS_switchOff = 1;
		}
	}
	HAL_TIM_Base_Stop_IT(&htim17);
  }

  /*if (htim == &htim16){ // 1 sec timer that checks if the CAN connections are all OK.
	  SO.canOverallStatus = SO_checkCanObservers(&SO);
	  if (SO.canOverallStatus != ALL_CANS_HEALTHY ) {
		  ME.ErrorFlag = 1;
		  ME_addErrors(&ME,ERR_SYSTEM_LEVEL_SOURCE,SYS_CAN_CUT_ERROR, SO.canOverallStatus, 0); // maybe later find out which ACK failed.
		  S.SMPS_switchOff = 1;
		  //stp the timer if you find you have an error.
		  HAL_TIM_Base_Stop_IT(&htim16);
	  }
  }*/

  if(htim==&htim15){
	  // This is meant to send motor run time data to the app when diag is running.
	  // here all motors run only 1 at a time
	  if (D.motorID <= 6){
		  uint8_t packetSize = BT_MC_generateDiagnosticMsg(D.motorID);
		  HAL_UART_Transmit_IT(&huart1,(uint8_t*)BufferTransmit,packetSize);
	  }
  }

  if(htim == &htim7){ //100ms timer for state sending to BT
	  timer7Count ++;
	  if (timer7Count % 5 == 0){
		  S.BT_sendState = 1;
		  S.TD_POT_check = 1;
		  if (S.runMode== RUN_ALL){
			  mcParams.currentMtrsRun += msp.delivery_mMin/60.0 * 0.5;
		  }
	  }

	  if (timer7Count == 10){
		  S.oneSecTimer++;
		  timer7Count = 0;
		  Toggle_State_LEDs(&S);
		  if(DR.requestSent){
			  DR.timer++;
		  }
		  if (sensor.ductTimerIncrementBool){
			  sensor.ductSensorTimer++;
		  }
	  }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	UNUSED(GPIO_Pin);

	if (GPIO_Pin == SMPS_OK_IP_Pin){
		// if the mother board has turned the SMPS on, but the smps is off, we have a problem!
		S.SMPS_OK_signal = (uint8_t)(HAL_GPIO_ReadPin(SMPS_OK_IP_GPIO_Port, SMPS_OK_IP_Pin));
		if ((S.SMPS_cntrl == SMPS_TURNEDON) && (S.SMPS_OK_signal == SMPS_OFF)){
			ME.ErrorFlag = 1;
			ME_addErrors(&ME,ERR_SYSTEM_LEVEL_SOURCE, SYS_SMPS_ERROR, ERROR_SOURCE_SYSTEM,0); // maybe later find out which ACK failed.
			S.SMPS_switchOff = 1;
		}
	}
	else{
		// for the user input buttons - RED_pin,Green_Pin,Yellow_Pin are falling edge interrupts. Rotary_Pin is both
		 usrBtns.initialState =(uint8_t)(HAL_GPIO_ReadPin(RED_GPIO_Port, GPIO_Pin));
		 if (GPIO_Pin == RED_Pin){
			  usrBtns.redBtn = usrBtns.initialState;
		  }
		  else if (GPIO_Pin == GREEN_Pin){
			  usrBtns.greenBtn = usrBtns.initialState;
		  }
		  else if (GPIO_Pin == YELLOW_Pin){
			  usrBtns.yellowBtn = usrBtns.initialState;
		  }else{
			  usrBtns.rotarySwitch = usrBtns.initialState;
		  }
	}
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {
    /* Retreive Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
    	FDCAN_parseForMotherBoard();
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
      /* Notification Error */
      Error_Handler();
    }
  }
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
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_DAC1_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_TIM15_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  //put all the HW setup first and then do the structs: hygiene!
  FDCAN_TxInit();
  FDCAN_RxFilterInit();

  //get the ADC for the tension draft pot ready
  if(HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK){
	  Error_Handler();
  }

  //Initialize custom typeDefs
  InitializeState_TypeDef(&S);
  InitializeRunTime_TypeDef();
  InitializeDiagnostic_TypeDef(&D);
  InitializeMotherBoardErrors_TypeDef(&MBE);//Todo : cleanup
  InitializeBTMsg_TypeDef(&BT);

  //now setup the Machine Settings
  ReadUserSettingsFromEeprom(&u);
  MBE.EepromLoadValsGood = CheckUserSettings(&u);
  if (MBE.EepromLoadValsGood == 0){
	  LoadDefaultUserSettings(&u);
	  //manualWrite
	  /*
	  u.delivery_mMin = 10;
	  u.lengthLimit = 100;
	  u.cardCylRPM = 750;
	  u.btrCylRPM = 600;
	  u.pickerCylRPM = 500;
	  u.deliveryMtrMin_CardFeed_Ratio=5.0f;
	  u.btrFeed_CardFeed_Ratio = 1.01;
	  u.afFeed_BtrFeed_Ratio=1.02;
	  */
	  MBE.defaults_eepromWriteFailed = WriteUserSettingsIntoEeprom(&u);
  }
  setupCardingMCType(&C,&u);
  ReadySetupCommand_AllMotors(&C);
  tdp.tensionDraft = C.tensionDraft; //initialize the POT

  //set up MCP chip
  mcp23017_init(&hmcp, &hi2c2, MCP23017_ADDRESS);
  mcp23017_setupForMainBoard(&hmcp);
  //set the initial Tower Lamp State
  TowerLamp_SetState(&hmcp, &mcp_portB,BUZZER_OFF,RED_OFF,GREEN_OFF,AMBER_OFF);
  TowerLamp_ApplyState(&hmcp,&mcp_portB);

  //read initial btns states
  UsrBtns_SetInitialStates();

  S.current_state = INITIAL_STATE;
  ChangeState(&S,IDLE_STATE);

  //Setup the Bluetooth device MANUALLY ONLY.
  BTCmd.manual_setup = 0;
  if (BTCmd.manual_setup){
	  BTCmd.manual_setup_result = BT_SetupDevice();
	  if(BTCmd.manual_setup_result != 1){
		 BTCmd.manual_setup_rebootOK = BTConsole_WriteCMD(BTCMD_REBOOT);
	  }
  }

  //at startup make both duct sensor states, instantaneous and current open
  //sensor.ductSensor = Sensor_ReadValueDirectly(&hmcp,&mcp_portB_sensorVal,DUCT_SENSOR);
  //sensor.ductCurrentState = sensor.ductSensor ;

  //set AC ssr Off
  setSSR_State(AC_SSR_OFF,&hmcp, &mcp_portA);
  S.AC_SSR_On = 0;

  //Interrupts on UART1 connected to the bluetooth
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//interrupt on receive buffer not empty(buffer full)
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_TC );//interrupt on Transmission Complete
  HAL_UART_Receive_DMA(&huart1,(uint8_t *)BufferRec,150);	 // start the receive here.
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_TC );//interrupt on Transmission Complete

  //Start the 100 msec Timer for sending BT state commands
  HAL_TIM_Base_Start_IT(&htim7);
  //Start the 1000 msec Timer for checking if the CAN bus is Ok
  HAL_TIM_Base_Start_IT(&htim16);

  //SMPS - turn on the SMPS, wait a while to see if  short command.
  SMPS_Init();
  HAL_Delay(1000);//contactor takes a long time to turn on.
  S.SMPS_cntrl = SMPS_TURNEDON;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	if (S.current_state == IDLE_STATE){
		IdleState();
	}

	if (S.current_state == RUN_STATE){
		RunState();
	}

	if (S.current_state == DEBUG_STATE){
		DebugState();
	}

	if (S.current_state == DIAGNOSTICS_STATE){
		DiagnosticsState();
	}

	if (S.current_state == SETTINGS_STATE){
		SettingsState();
	}

	if (S.current_state == ERROR_STATE){
		ErrorState();
	}

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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
  sConfig.DAC_DMADoubleDataMode = DISABLE;
  sConfig.DAC_SignedFormat = DISABLE;
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_EXTERNAL;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV10;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = ENABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 6;
  hfdcan1.Init.NominalTimeSeg1 = 23;
  hfdcan1.Init.NominalTimeSeg2 = 6;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.DataTimeSeg1 = 11;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x50D4E6FF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1499;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1499;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 9999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1499;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 24999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 29999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 4999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */
//2uS timer
  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 1499;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1499;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT|UART_ADVFEATURE_RXOVERRUNDISABLE_INIT
                              |UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 2000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_CNTRL_GND_Pin|EXTRA1_Pin|DBG_TP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SMPS_CNTRL_OP_Pin|EXTRA5_Pin|EXTRA7_Pin|EXTRA4_Pin
                          |EXTRA3_Pin|EXTRA2_Pin|CAN_LED_Pin|FAULT_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_Pin YELLOW_Pin RED_Pin ROTARY_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin|YELLOW_Pin|RED_Pin|ROTARY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_OK_IP_Pin */
  GPIO_InitStruct.Pin = SMPS_OK_IP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SMPS_OK_IP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_CNTRL_GND_Pin EXTRA1_Pin DBG_TP_Pin */
  GPIO_InitStruct.Pin = SMPS_CNTRL_GND_Pin|EXTRA1_Pin|DBG_TP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_CNTRL_OP_Pin EXTRA5_Pin EXTRA7_Pin EXTRA4_Pin
                           EXTRA3_Pin EXTRA2_Pin CAN_LED_Pin FAULT_LED_Pin */
  GPIO_InitStruct.Pin = SMPS_CNTRL_OP_Pin|EXTRA5_Pin|EXTRA7_Pin|EXTRA4_Pin
                          |EXTRA3_Pin|EXTRA2_Pin|CAN_LED_Pin|FAULT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
	//TODO here send some error Code to the App.
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
