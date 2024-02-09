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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stm32l4xx_hal.h"
#include "crc32.h"
#define CBC 1
#include "aes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAJOR 0		// BL Major version Number
#define MINOR 1		// BL Minor version Number
#define FW_SIZE 470
#define FLASH_CONF_ADDRESS 0x08020000
#define FLASH_APP_ADDRESS1 0x08030000
#define FLASH_APP_ADDRESS2 0x08058000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint8_t BL_Version[2] = { MAJOR, MINOR };
uint8_t key[] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
uint8_t iv[]  = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f };
struct AES_ctx ctx;
uint8_t rx_Slavebuff[71];
uint8_t buffTrigger = 1;
uint8_t configuration[8];
uint8_t application[8];
uint16_t PacketCounter=0;
uint8_t buffCounter4 = 0;
uint8_t rx_Tbuff[20];
uint8_t x_buff4[20];
uint8_t ack_pack[2] = "AK";
static uint8_t ackFlag = 0;
static uint8_t OTA_conf[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef write_cfg_to_flash(uint8_t *ulldata, uint32_t data_len );
static HAL_StatusTypeDef write_data_to_flash(uint8_t *ulldata, uint32_t data_len );
static void goto_application(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  uint8_t * conf = (uint8_t *) FLASH_CONF_ADDRESS;
  memcpy(configuration, conf, 8);

  uint8_t * app = (uint8_t *) FLASH_APP_ADDRESS1;
  if (configuration[3] == 0x01){
	  app = (uint8_t *) FLASH_APP_ADDRESS1; //0x08030000;
  }
  else if (configuration[3] == 0x02){
	  app = (uint8_t *) FLASH_APP_ADDRESS2;	//0x08058000
  }

  memcpy(application, app, 8);

  if (configuration[0]== 0xaa){
	  HAL_UART_Transmit(&huart4, ack_pack, sizeof(ack_pack), 100);
	  ackFlag = 1;
	  configuration[0] = 0xee;
	  write_cfg_to_flash(configuration, sizeof(configuration));
  }
  if (configuration[0]== 0xff || configuration[0]== 0xbb){
	  buffTrigger = 1;
  }
  else{
	  buffTrigger = 71;
  }
  HAL_UART_Receive_IT(&huart4, rx_Slavebuff, buffTrigger);
  HAL_UART_Receive_IT(&huart2, rx_Tbuff, 1);


  printf("Starting RxBootLoader(%d.%d)\r\n", BL_Version[0], BL_Version[1] );
  printf("%d %x %x %x %x", ackFlag, configuration[0], configuration[1], configuration[2], configuration[3]);

  if (configuration[0] == 0xbb){
	  goto_application();
  }
  else if (configuration[3] == 0x02 && ackFlag == 0 && configuration[1] == 0xDD){
	  configuration[3] = 0x01;
	  goto_application();
  }
  else if (configuration[3] == 0x01 && ackFlag == 0 && configuration[2] == 0xDD){
	  configuration[3] = 0x02;
	  goto_application();
  }
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
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS_EN_Pin SMPS_V1_Pin SMPS_SW_Pin */
  GPIO_InitStruct.Pin = SMPS_EN_Pin|SMPS_V1_Pin|SMPS_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SMPS_PG_Pin */
  GPIO_InitStruct.Pin = SMPS_PG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SMPS_PG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD4_Pin */
  GPIO_InitStruct.Pin = LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD4_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2){
		HAL_UART_Transmit(&huart2, rx_Tbuff, 1, 10);
		HAL_UART_Receive_IT(&huart2, rx_Tbuff, 1);

		if (rx_Tbuff[0]=='#'){
			printf("\n\r");
			goto_application();
		}
	}
	else if (huart == &huart4){
		uint8_t * conf = (uint8_t *) FLASH_CONF_ADDRESS;
		memcpy(OTA_conf, conf, 8);
		OTA_conf[0] = 0xBB;

		x_buff4[buffCounter4] = rx_Slavebuff[0];

		if (rx_Slavebuff[0] == '!' && buffTrigger == 1){
			printf("OTA REQUEST RECIEVED \r\n");
			HAL_UART_Transmit(&huart4, ack_pack, sizeof(ack_pack), 100);
			buffCounter4 = 0;
			buffTrigger = 71;
		}
		else if (rx_Slavebuff[0] == 'K' && x_buff4[buffCounter4-1] == 'O'){
			HAL_UART_Transmit(&huart2, x_buff4, sizeof(x_buff4), 100);
		}
		else if (buffTrigger == 71){
//			AES_ECB_decrypt(rx_Slavebuff, key, decryptOTAData, 64);
			static uint8_t pkt_ack[2] = "@@";
			if (PacketCounter == 0){
				start(0);
			}
			printf("Packet Number Received %d \n\r", PacketCounter++);
			update(rx_Slavebuff, 64);
			AES_init_ctx_iv(&ctx, key, iv);
			AES_CBC_decrypt_buffer(&ctx, rx_Slavebuff, 64);
			write_data_to_flash(rx_Slavebuff, 64);
			HAL_UART_Transmit(&huart4, pkt_ack, 1, 100);
			printf("ACK Sent \r\n");
			if (rx_Slavebuff[66] == 0xf2){
				uint32_t cal_crc = finish();
				printf("CRC RxVALUE %lu \r\n", cal_crc);
				if (memcmp(&rx_Slavebuff[67], &cal_crc, 4) == 0){
					printf("CRC MATCHED \r\n");
				}else{
					printf("CRC MATCH FAILED \r\n");
					pkt_ack[0] = '$';
					pkt_ack[1] = '$';
				}
				printf("Configuration Update: ");
				if (configuration[3] == 0xFF || configuration[3] == 0x01){
					OTA_conf[3] = 0x01;
					OTA_conf[1] = 0xDD;
					printf("Case 1 \r\n");
				}
				else if (configuration[3] == 0x02){
					OTA_conf[3] = 0x02;
					OTA_conf[2] = 0xDD;
					printf("Case 2 \r\n");
				}
				printf("OTA COMPLETED\r\n");
				ackFlag = 0;
				write_cfg_to_flash(OTA_conf, sizeof(OTA_conf));
//				buffTrigger = 1;
				HAL_NVIC_SystemReset();
			}
		}
		HAL_UART_Receive_IT(&huart4, rx_Slavebuff, buffTrigger);
	}

}



static HAL_StatusTypeDef write_data_to_flash(uint8_t *ulldata, uint32_t data_len )
{
  HAL_StatusTypeDef ret;
  static uint32_t address = FLASH_APP_ADDRESS1;	//0x08030000;
  static uint8_t flashFlag = 0;
  static uint32_t pageAdress;

  if ((configuration[1] == 0xFF || configuration[3] == 0x01) && flashFlag == 0){
	  address = FLASH_APP_ADDRESS1; //0x08030000;
	  flashFlag = 1;
	  pageAdress = 96; // 1 Page = 2K, So 96 = (0x3 0000)
	  printf("Writing data in Slot 1\r\n");
  }
  else if ((configuration[2] == 0xFF || configuration[3] == 0x02) && flashFlag == 0){
	  address = FLASH_APP_ADDRESS2;	//0x08058000
	  flashFlag = 1;
	  pageAdress = 176; // 1 Page = 2K, So 176 = (0x5 8000)
	  printf("Writing data in Slot 2\r\n");
  }

  static uint8_t eraseFlag = 0;

  do
  {
    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK )
    {
      break;
    }

    //Check if the FLASH_FLAG_BSY.
    FLASH_WaitForLastOperation(HAL_MAX_DELAY);


    if (application[0] != 0xff && eraseFlag ==0){
    	printf("Erasing the App Flash memory...\r\n");
		//Erase the Flash
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t SectorError;

		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Banks       = FLASH_BANK_1;
		EraseInitStruct.Page        = pageAdress;
		EraseInitStruct.NbPages     = 75;

		ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
		if( ret != HAL_OK )
		{
		  printf("Flash erase Error\r\n");
		  break;
		}
		printf("Erased the App Flash memory...\r\n");
		eraseFlag = 1;

    }


    for(uint32_t i = 0; i < data_len; i+=8)
    {
      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD,
                               (address),
							   *(uint64_t*)&ulldata[i]);

      address += 8;

      if( ret != HAL_OK )
      {
        printf("App Flash Write Error\r\n");
        break;
      }
    }

    if( ret != HAL_OK )
    {
      break;
    }

    ret = HAL_FLASH_Lock();
    if( ret != HAL_OK )
    {
      break;
    }

    //Check if the FLASH_FLAG_BSY.
    FLASH_WaitForLastOperation(HAL_MAX_DELAY);

  }while(0);

  return ret;
}



static void goto_application(void)
{
	static uint32_t App_address = FLASH_APP_ADDRESS1;//0x08030000;
	if (configuration[1] == 0xDD && configuration[3] == 0x01){
		App_address = FLASH_APP_ADDRESS1;//0x08030000;
//		write_cfg_to_flash(configuration, sizeof(configuration));
		printf("Jumping to Application in Slot1\r\n");
	}
	else if (configuration[2] == 0xDD && configuration[3] == 0x02){
		App_address = FLASH_APP_ADDRESS2;	//0x08058000;
//		write_cfg_to_flash(configuration, sizeof(configuration));
		printf("Jumping to Application in Slot2\r\n");
	}


  void (*app_reset_handler)(void) = (void*)(*((volatile uint32_t*) (App_address + 4)));
  //__set_MSP(*(volatile uint32_t*) 0x08030000);
  // Turn OFF the Green Led to tell the user that Bootloader is not running
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET );    //Green LED OFF
  app_reset_handler();    //call the app reset handler
}



static HAL_StatusTypeDef write_cfg_to_flash(uint8_t *ulldata, uint32_t data_len )
{
  HAL_StatusTypeDef ret;
  static uint32_t address = FLASH_CONF_ADDRESS;//0x08020000;

  do
  {
    ret = HAL_FLASH_Unlock();
    if( ret != HAL_OK )
    {
      break;
    }

    //Check if the FLASH_FLAG_BSY.
    FLASH_WaitForLastOperation(HAL_MAX_DELAY);


    // clear all flags before you write it to flash
//    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
//                FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR);

    if ( configuration[0] != 0xff){
    	printf("Erasing the Config Flash memory...\r\n");
		//Erase the Flash
		FLASH_EraseInitTypeDef EraseInitStruct;
		uint32_t SectorError;

		EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Banks     = FLASH_BANK_1;
		EraseInitStruct.Page        = 64; // 1 page = 2K
		EraseInitStruct.NbPages     = 2;                    //erase 2 sectors(5,6)
	//    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;

		ret = HAL_FLASHEx_Erase( &EraseInitStruct, &SectorError );
		if( ret != HAL_OK )
		{
		  printf("Flash erase Error\r\n");
		  break;
		}
    }


    for(uint32_t i = 0; i < data_len; i+=8)
    {
      ret = HAL_FLASH_Program( FLASH_TYPEPROGRAM_DOUBLEWORD,
                               (address),
							   *(uint64_t*)&ulldata[i]);

      if( ret != HAL_OK )
      {
        printf("App Flash Write Error\r\n");
        break;
      }
    }

    if( ret != HAL_OK )
    {
      break;
    }

    ret = HAL_FLASH_Lock();
    if( ret != HAL_OK )
    {
      break;
    }

    //Check if the FLASH_FLAG_BSY.
    FLASH_WaitForLastOperation(HAL_MAX_DELAY);

  }while(0);

  return ret;
}


#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the UART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
