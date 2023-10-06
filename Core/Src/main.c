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
#include "stk500.h"
#include "stm32f1xx_ll_flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*p_APP)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OPTIBOOT_MAJVER 4
#define OPTIBOOT_MINVER 5

#define SIGNATURE_0 0x1E
#define SIGNATURE_1 0x55 // 0x97
#define SIGNATURE_2 0xAA // 0x02

#define SIGNATURE_3 0x97
#define SIGNATURE_4 0x02
#define APP_ADDR 0x08001000
#define FLASH_SIZE 0x08020000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t Port;
uint8_t Buff[512];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void runApp();
uint8_t getch(void);
void putch(uint8_t byte);
uint8_t verifySpace(void);
void bgetNch(uint8_t count);

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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  LL_Flash_Unlock();

  uint8_t check;
  check = LL_RCC_IsActiveFlag_SFTRST();
  LL_RCC_ClearResetFlags();
  
  if (check)
  {
    LL_mDelay(150);
    if (0xF0 != (LL_GPIO_ReadInputPort(GPIOA) & 0xF1))
      runApp();
  }

  LL_TIM_EnableCounter(TIM2);

  uint8_t ch = 0;
  uint8_t lastCh = 0;
  uint8_t NotSynced = 1;
  uint32_t address = 0;

  uint8_t *bufPtr;
  uint16_t length;
  uint16_t count;
  uint16_t data;
  uint8_t *memAddress;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    while (NotSynced)
    {
      if (LL_USART_IsActiveFlag_RXNE(USART2))
      {
        ch = LL_USART_ReceiveData8(USART2);
        Port = 0;
      }
      if (LL_USART_IsActiveFlag_RXNE(USART1))
      {
        ch = LL_USART_ReceiveData8(USART1);
        Port = 1;
      }
      if ((lastCh == STK_GET_SYNC) && (ch == CRC_EOP))
      {
        NotSynced = 0;
        break;
      }
      lastCh = ch;
    }

    ch = getch();
    switch (ch)
    {
    case STK_GET_PARAMETER:
      lastCh = getch();
      NotSynced = verifySpace();
      if (lastCh == 0x82)
      {
        putch(OPTIBOOT_MINVER);
      }
      else if (lastCh == 0x81)
      {
        putch(OPTIBOOT_MAJVER);
      }
      else
        putch(0x03);
      break;

    case STK_SET_DEVICE:
      bgetNch(20);
      NotSynced = verifySpace();
      break;

    case STK_SET_DEVICE_EXT:
      bgetNch(5);
      NotSynced = verifySpace();
      break;

    case STK_LOAD_ADDRESS:
      address = getch();
      address = (address & 0xff) | (getch() << 8);
      address <<= 1;
      NotSynced = verifySpace();
      break;

    case STK_UNIVERSAL:
      bgetNch(4);
      NotSynced = verifySpace();
      putch(0x00);
      break;

    case STK_PROG_PAGE:
    {

      memAddress = (uint8_t *)(address + 0x08000000);

      length = getch() << 8; /* getlen() */
      length |= getch();
      getch(); // discard flash/eeprom byte
      // While that is going on, read in page contents
      count = length;
      bufPtr = Buff;
      do
      {
        *bufPtr++ = getch();
      } while (--count);
      if (length & 1)
      {
        *bufPtr = 0xFF;
      }
      count = length;
      count += 1;
      count /= 2;

      NotSynced = verifySpace();
      if (((uint32_t)memAddress < FLASH_SIZE) && ((uint32_t)memAddress >= APP_ADDR))
      {

        if (((uint32_t)memAddress & 0x000003FF) == 0)
        {
          // At page start so erase it
          // FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
          // FLASH_ErasePage((uint32_t)memAddress);
          LL_Flash_PageErase((uint32_t)memAddress);
        }
        bufPtr = Buff;
        while (count)
        {
          data = *bufPtr++;
          data |= *bufPtr++ << 8;
          // FLASH_ProgramHalfWord((uint32_t)memAddress, data);
          LL_FLASH_Program_TwoBtye((uint32_t)memAddress, data);
          memAddress += 2;
          count -= 1;
        }
      }
    }
    break;

    case STK_READ_PAGE:
    {
      memAddress = (uint8_t *)(address + 0x08000000);
      // READ PAGE - we only read flash
      length = getch() << 8; /* getlen() */
      length |= getch();
      getch();
      NotSynced = verifySpace();
      do
      {
        putch(*memAddress++);
      } while (--length);
    }
    break;

    case STK_READ_SIGN:

      NotSynced = verifySpace();
      putch(SIGNATURE_0);
      putch(SIGNATURE_1);
      putch(SIGNATURE_2);
      break;

    default:
      NotSynced = verifySpace();
      break;
    }
    if (NotSynced)
      continue;
    putch(STK_OK);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

  /* Wait till HSE is ready */
  while (LL_RCC_HSE_IsReady() != 1)
  {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  while (LL_RCC_PLL_IsReady() != 1)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
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

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

  /* TIM2 interrupt Init */
  NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 10, 0));
  NVIC_EnableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TIM_InitStruct.Prescaler = 71;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 49999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM2, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM2);
  LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM2);
  /* USER CODE BEGIN TIM2_Init 2 */
  LL_TIM_ClearFlag_UPDATE(TIM2);
  LL_TIM_EnableIT_UPDATE(TIM2);
  /* USER CODE END TIM2_Init 2 */
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 57600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 57600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  LL_USART_Enable(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**USART3 GPIO Configuration
  PB10   ------> USART3_TX
  PB11   ------> USART3_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  USART_InitStruct.BaudRate = 57600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART3, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART3);
  LL_USART_Enable(USART3);
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin);

  /**/
  LL_GPIO_SetOutputPin(INV1_GPIO_Port, INV1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(INV2_GPIO_Port, INV2_Pin);

  /**/
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = S1_Pin | S2_Pin | S3_Pin | S4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = INV2_Pin | INV1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void runApp()
{

  if (((*(__IO uint32_t *)APP_ADDR) & 0x2ffe0000) == 0x20000000)
  {
    p_APP application;
    uint32_t jump_address;
    jump_address = *(__IO uint32_t *)(APP_ADDR + 4);
    application = (p_APP)jump_address;
    __disable_irq();

    LL_USART_DeInit(USART1);
    LL_USART_DeInit(USART2);
    LL_USART_DeInit(USART3);

    NVIC_DisableIRQ(TIM2_IRQn);
    LL_TIM_DeInit(TIM2);

    LL_GPIO_DeInit(GPIOA);
    LL_GPIO_DeInit(GPIOB);

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    SCB->VTOR = APP_ADDR;
    __set_MSP(*(__IO uint32_t *)APP_ADDR);
    application();
  }
}

uint8_t getch(void)
{
  if (Port)
  {
    while (LL_USART_IsActiveFlag_RXNE(USART1) == 0)
      ;
    return LL_USART_ReceiveData8(USART1);
  }
  else
  {
    while (LL_USART_IsActiveFlag_RXNE(USART2) == 0)
      ;
    return LL_USART_ReceiveData8(USART2);
  }
}

void putch(uint8_t byte)
{
  if (Port)
  {
    while (LL_USART_IsActiveFlag_TXE(USART1) == 0)
      ;
    LL_USART_TransmitData8(USART1, byte);
  }
  else
  {
    while (LL_USART_IsActiveFlag_TXE(USART3) == 0)
      ;
    LL_USART_TransmitData8(USART3, byte);
  }
}

uint8_t verifySpace(void)
{
  if (getch() != CRC_EOP)
  {
    return 1;
  }
  putch(STK_INSYNC);
  return 0;
}

void bgetNch(uint8_t count)
{
  do
  {
    getch();
  } while (--count);
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
