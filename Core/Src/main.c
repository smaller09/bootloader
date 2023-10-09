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
#define APP_ADDR 0x08000800
#define FLASH_SIZE 0x08020000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t Buff[512];
uint8_t LED = 0;
__IO uint32_t *G_SR; // for getch
__IO uint32_t *G_DR;

__IO uint32_t *P_SR; // for putch
__IO uint32_t *P_DR;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void runApp();
uint8_t getch(void);
void putch(uint8_t byte);
uint8_t verifySpace(void);
void bgetNch(uint8_t count);
void system_init(void);
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

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO | LL_APB2_GRP1_PERIPH_USART1 | LL_APB2_GRP1_PERIPH_GPIOA | LL_APB2_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR | LL_APB1_GRP1_PERIPH_USART2 | LL_APB1_GRP1_PERIPH_USART3);

    /* System interrupt init*/
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* SysTick_IRQn interrupt configuration */
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    system_init();
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    /* USER CODE BEGIN 2 */
    G_SR = (__IO uint32_t *)(USART2_BASE);
    G_DR = (__IO uint32_t *)(USART2_BASE + 4);
    P_SR = (__IO uint32_t *)(USART3_BASE);
    P_DR = (__IO uint32_t *)(USART3_BASE + 4);
    LL_Flash_Unlock();

    if (LL_RCC_IsActiveFlag_SFTRST() == 0)
    {
        LL_mDelay(150);
        if (0xF0 != ((uint8_t)(GPIOA->IDR) & 0xF1))
        {
            runApp();
        }
    }
    LL_RCC_ClearResetFlags();

    LED = 1;
    uint8_t ch = 0;
    uint8_t lastCh = 0;
    uint8_t NotSynced = 1;
    uint32_t address = 0;
    uint8_t *bufPtr;
    uint16_t length;
    uint16_t count;
    uint16_t data;
    uint8_t *memAddress;
    uint8_t Port;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        while (NotSynced)
        {

            if (USART2->SR & USART_SR_RXNE)
            {
                ch = USART2->DR;
                Port = 0;
            }
            if (USART1->SR & USART_SR_RXNE)
            {
                ch = USART2->DR;
                Port = 1;
            }
            if ((lastCh == STK_GET_SYNC) && (ch == CRC_EOP))
            {
                NotSynced = 0;
                break;
            }
            lastCh = ch;
        }

        if (Port)
        {
            G_SR = (__IO uint32_t *)(USART1_BASE);
            G_DR = (__IO uint32_t *)(USART1_BASE + 4);
            P_SR = (__IO uint32_t *)(USART1_BASE);
            P_DR = (__IO uint32_t *)(USART1_BASE + 4);
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
            count = length + 1;
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
                    count--;
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
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
    {
    }
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1)
    {
    }
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while (LL_RCC_PLL_IsReady() != 1)
    {
    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
    }
    LL_Init1msTick(36000000);
    LL_SetSystemCoreClock(36000000);
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

        /* Force reset of USART clock */
        LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_USART2 | LL_APB1_GRP1_PERIPH_USART3);

        /* Release reset of USART clock */
        LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_USART2 | LL_APB1_GRP1_PERIPH_USART3);

        LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_GPIOA | LL_APB2_GRP1_PERIPH_GPIOB | LL_APB2_GRP1_PERIPH_USART1);
        LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_GPIOA | LL_APB2_GRP1_PERIPH_GPIOB | LL_APB2_GRP1_PERIPH_USART1);

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
    while ((*G_SR & USART_SR_RXNE) == 0)
    {
    }

    return *G_DR;
}

void putch(uint8_t byte)
{
    while ((*P_SR & USART_SR_TXE) == 0)
    {
    }
    *P_DR = byte;
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

void system_init(void)
{
    GPIOA->CRH = (GPIOA->CRH & 0xFFFFFF0F) | 0x00000090; // PA9
    // USART2 RX is PA3, only Rx used
    // USART3 TX is PB10, only Tx used
    GPIOB->CRH = (GPIOB->CRH & 0xFFFFF0FF) | 0x00000900; // PB10

    GPIOB->CRL = (GPIOB->CRL & 0xFFFF0F0F) | 0x00002020; // PB1 and PB3, invert controls

    GPIOB->BRR = 0x00000008;
    GPIOB->BSRR = 0x00000002;

    GPIOA->BSRR = 0x000000F1;
    GPIOA->CRL = (GPIOA->CRL & 0x0000FF00) | 0x88880028; // LED and inputs

    USART1->BRR = 36000000 / 57600;
    USART1->CR1 = 0x200C;
    USART2->BRR = 36000000 / 57600;
    USART2->CR1 = 0x200C;
    // USART2->CR2 = 0;
    // USART2->CR3 = 0;
    USART3->BRR = 36000000 / 57600;
    USART3->CR1 = 0x200C;
    USART3->CR2 = 0;
    USART3->CR3 = 0;
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
