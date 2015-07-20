/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 01/07/2015 07:25:48
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//#include <sys/stat.h>
#include <stdarg.h>
#include <stdio.h>
#include "stm32l0xx_hal.h"
#include "usb_device.h"

    #include <usbd_core.h>
    #include <usbd_cdc.h>
    #include <usbd_cdc_if.h>
    #include <usbd_desc.h>

#ifdef __cplusplus
extern "C"
#endif

#define CMD_BUFFER_LEN 64


extern char g_VCPInitialized;

typedef struct s_cmd {
  const char* cmd_text;
  int cmd_id;
} t_cmd;

#define CMD_GPIO_TEST 	1
#define CMD_I2C_TEST 	2
#define CMD_ADC_TEST	3
#define CMD_DAC_TEST	4
#define CMD_LED_TEST	5
#define CMD_HEAT_TEST	6

t_cmd hw_test_main_menu[] = {
  {"GPIO test", CMD_GPIO_TEST},
  {"I2C test", CMD_I2C_TEST},
  {"ADC test", CMD_ADC_TEST},
  {"DAC test", CMD_DAC_TEST},
  {"LED test", CMD_LED_TEST},
  {"Heat test", CMD_HEAT_TEST},
};
	


  //  USBD_HandleTypeDef USBD_Device;
//caddr_t _sbrk(int increment);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void usbPrintf(const char* lpszFormat, ...);
int usbRead(char *pBuffer, int size);
void printCmdPrompt(char* prefix, t_cmd menu[], int numOfMenuitem);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


int main(void)
{

  /* USER CODE BEGIN 1 */
  char byte;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  
 /*   USBD_Init(&USBD_Device, &FS_Desc, 0);

    USBD_RegisterClass(&USBD_Device, &USBD_CDC);
    USBD_CDC_RegisterInterface(&USBD_Device,
                               &USBD_Interface_fops_FS);
    USBD_Start(&USBD_Device);
*/
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (!g_ComPortOpen);
    usbPrintf("\r\nHareware Test Menu: \r\n");

#if 1
    while(1) {
      printCmdPrompt("", hw_test_main_menu, sizeof(hw_test_main_menu) / sizeof(hw_test_main_menu[0]));

      usbPrintf("Please enter number to select test item: ");
      usbRead(&byte, 1);
      switch(byte) {
      case CMD_GPIO_TEST:
        usbPrintf(hw_test_main_menu[0].cmd_text);
        break;
      case CMD_I2C_TEST:
        usbPrintf(hw_test_main_menu[1].cmd_text);
        break;
      case CMD_ADC_TEST:
        usbPrintf(hw_test_main_menu[2].cmd_text);
        break;
      case CMD_DAC_TEST:
        usbPrintf(hw_test_main_menu[3].cmd_text);
        break;
      case CMD_LED_TEST:
        usbPrintf(hw_test_main_menu[4].cmd_text);
        break;
      case CMD_HEAT_TEST:
        usbPrintf(hw_test_main_menu[5].cmd_text);
        break;
      default:
        usbPrintf("Not a valid number\r\n");
      }
    }
    
    
      
    
    
  for (;;)
  {
    //VCP_write("Enter a number: \r\n", 17);
      if (VCP_read(&byte, 1) != 1)
          continue;
      
      usbPrintf("\r\nYou typed %c\r\n", byte);
      //usbPrintf(&byte, 1);
      //VCP_write("\r\n", 2);
      //usbPrintf("\r\n", 2);
      
  }
#else
  for (;;)
  {
      int val = 0;
      VCP_write("Enter a number: ");
      VCP_read(&val, 1);
      VCP_write("%d = 0x%x\r\n", val, val);
  }
#endif
}

void usbPrintf(const char* lpszFormat, ...)
{
  int nLen;
  char szBuffer[CMD_BUFFER_LEN + 1];
  va_list args;
  va_start(args, lpszFormat);
  nLen = vsnprintf(szBuffer, CMD_BUFFER_LEN + 1, lpszFormat, args);
  VCP_write(szBuffer, nLen);
  va_end(args);
}

int usbRead(char *pBuffer, int size)
{
  for (;;) {
    int done = VCP_read(pBuffer, size);
    if (done)
        return done;
  }
}

void printCmdPrompt(char* prefix, t_cmd menu[], int menu_num)
{
  int i;
  for (i = 0; i < menu_num; i++)
    usbPrintf("%s%d: %s\r\n", prefix, menu[i].cmd_id, menu[i].cmd_text);
}
#if 0
caddr_t _sbrk(int increment)
{
    extern char end asm("end");
    register char *pStack asm("sp");

    static char *s_pHeapEnd;

    if (!s_pHeapEnd)
        s_pHeapEnd = &end;

    if (s_pHeapEnd + increment > pStack)
        return (caddr_t)-1;

    char *pOldHeapEnd = s_pHeapEnd;
    s_pHeapEnd += increment;
    return (caddr_t)pOldHeapEnd;
}
#endif

extern PCD_HandleTypeDef hpcd;

void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}
    
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USB;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __SYSCFG_CLK_ENABLE();

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_7B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** Pinout Configuration
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
