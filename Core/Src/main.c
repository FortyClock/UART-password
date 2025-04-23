/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/*#define BUFFER_SIZE 16
volatile uint8_t uart_rx_buffer[BUFFER_SIZE];
volatile uint8_t uart_rx_index = 0;
volatile uint8_t uart_rx_ready = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile char uart_rx_buffer[5] = {0}; // Буфер для 4 символов + нуль-терминатор
volatile uint8_t uart_rx_index = 0;
volatile uint8_t uart_rx_ready = 0;

// В секции USER CODE BEGIN 4 (прерывание)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define FLASH_PAGE_SIZE         (0x400)     // 1024 байта
// Базовый адрес Flash (0x08000000)
#define FLASH_BASE_ADDR         (0x08000000UL)
// Адрес последней страницы (например, Page 30)
#define FLASH_USER_START_ADDR   (FLASH_BASE_ADDR + (31 * FLASH_PAGE_SIZE))

void Flash_Write_HalfWord(uint32_t address, uint16_t data) {
    HAL_StatusTypeDef status;
    uint32_t pageError = 0;
    FLASH_EraseInitTypeDef eraseInitStruct;

    // Разблокировка Flash
    HAL_FLASH_Unlock();

    // Настройка структуры для стирания страницы
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    eraseInitStruct.NbPages = 1;  // Стираем только одну страницу

    // Стирание страницы
    status = HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();  // Блокировка Flash в случае ошибки
        return;
    }

    // Запись 16-битного слова (полуслова)
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data);
    if (status != HAL_OK) {
        // Обработка ошибки записи (например, вывод в UART)
    }


    // Блокировка Flash
    HAL_FLASH_Lock();
}

void Read_From_Flash(uint32_t address, uint16_t* data) {
        *data = *(__IO uint16_t*)address;
    }

void UART_Receive_Callback(uint8_t received_char) {
	if(received_char == '\r' || received_char == '\n') {
		uart_rx_buffer[uart_rx_index] = '\0';
		uart_rx_ready = 1;
		uart_rx_index = 0;
	} else if(uart_rx_index < 5-1) {
		uart_rx_buffer[uart_rx_index++] = received_char;
	}
}

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_buffer, 1);
  uint32_t address = FLASH_USER_START_ADDR; // Адрес страницы 30
  uint32_t data = 0xACAB; // Данные для записи (16 бит)

  Flash_Write_HalfWord(address, data);


  uint16_t flash_data;
  uint16_t user_value;
  char message[50];

  Read_From_Flash(FLASH_USER_START_ADDR, &flash_data); // чтение данных ячейки

  const char* prompt = "Enter the val from Flash (hex, 4 symbols):\r\n";
  HAL_UART_Transmit(&huart1, (uint8_t*)prompt, strlen(prompt), HAL_MAX_DELAY); // отправка приглащения

  // прием данных
  /*while(!uart_rx_ready) {
         uint8_t byte;
         if(HAL_UART_Receive(&huart1, &byte, 1, 100) == HAL_OK) {
             UART_Receive_Callback(byte);
             HAL_UART_Transmit(&huart1, &byte, 1, 100); // Эхо-вывод
         }
  }
*/
   HAL_UART_Receive_IT(&huart1, uart_rx_buffer, 1);

   uint16_t flash_value;
   Read_From_Flash(FLASH_USER_START_ADDR, &flash_value);

   while (1) {
          if (uart_rx_ready) {
              uart_rx_buffer[5] = '\0'; // Null-terminate the string
              user_value = (uint16_t)strtoul((char*)uart_rx_buffer, NULL, 16);

              if (user_value == flash_data) {
                  HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nGood pass!\r\n", 15, HAL_MAX_DELAY);
              } else {
                  HAL_UART_Transmit(&huart1, (uint8_t*)"\r\nBadPass!\r\n", 14, HAL_MAX_DELAY);
              }

              memset(uart_rx_buffer, 0, 5);
              uart_rx_index = 0;
              uart_rx_ready = 0;
              HAL_UART_Transmit(&huart1, (uint8_t*)prompt, strlen(prompt), HAL_MAX_DELAY);
              HAL_UART_Receive_IT(&huart1, uart_rx_buffer, 1); // Restart прием
          }
      }
  }
/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        if(uart_rx_index < 4 && !uart_rx_ready) {
            char received = uart_rx_buffer[uart_rx_index];

            if(received >= '0' && received <= '9' ||
               received >= 'A' && received <= 'F' ||
               received >= 'a' && received <= 'f') {
                uart_rx_buffer[uart_rx_index++] = received;
                HAL_UART_Transmit(&huart1, (uint8_t*)&received, 1, 10); // Эхо
            }

            if(uart_rx_index == 4) {
                uart_rx_buffer[4] = '\0';
                uart_rx_ready = 1;
            }
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart_rx_buffer[uart_rx_index], 1);
    }
}

/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart->Instance == USART1) {
        UART_Receive_Callback(uart_rx_buffer[0]);
    }
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
