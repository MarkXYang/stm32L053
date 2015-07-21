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

#include "main.h"

#ifdef __cplusplus
extern "C"
#endif

#define I2C_ADDRESS             0x90

extern char g_VCPInitialized;


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
static GPIO_InitTypeDef  GPIO_InitStruct;
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */



/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */


int main(void)
{

  /* USER CODE BEGIN 1 */
  char byte;
  int iByte;
  

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  
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
    
  printCmdPrompt("", hw_test_main_menu, sizeof(hw_test_main_menu) / sizeof(hw_test_main_menu[0]));
  usbPrintf("Please enter number to select test item: ");

#if 1
    while(1) {
      
      usbRead(&byte, 1);
      usbPrintf(&byte);
      iByte = char2int(byte);
      switch(iByte) {
      case CMD_GPIO_TEST:
        gpio_test();
        break;
      case CMD_I2C_TEST:
        i2c_test();
        break;
      case CMD_ADC_TEST:
        usbPrintf(hw_test_main_menu[iByte].cmd_text);
        usbPrintf("\r\n");
        break;
      case CMD_DAC_TEST:
        usbPrintf(hw_test_main_menu[iByte].cmd_text);
        usbPrintf("\r\n");
        break;
      case CMD_LED_TEST:
        usbPrintf(hw_test_main_menu[iByte].cmd_text);
        usbPrintf("\r\n");
        break;
      case CMD_HEAT_TEST:
        usbPrintf(hw_test_main_menu[iByte].cmd_text);
        usbPrintf("\r\n");
        break;
      default:
        usbPrintf("Not a valid number\r\n");
      }
      usbPrintf("Please enter number to select test item: ");
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

int char2int(char byte)
{
  int i = byte - 0x30;
  if (i < 0)            /*Check if 0 <= i < 10 */
    return -1;
  else if ( i < 10)
    return i;
  else {
    i = byte - 0x41;
    if (i >= 0 && i < 6) /* upper case A to F */
      return i + 10;
    else {
      i = byte - 0x61;
      if (i >= 0 && i < 6) /* low case a to f */
        return i + 10;
      else
        return -1;
    }
  }
}

int gpio_test(void)
{
  char iBank, cGpioPin, gpioDir, cGpioOutVal;
  int nGpioPin, nGpioOutVal;
  GPIO_TypeDef* gpioBank;
  
  usbPrintf("\r\nPlease input GPIO Bank(A-B): ");
  usbRead(&iBank, 1);
  usbPrintf(&iBank);
  if(iBank == 'A' | iBank == 'a') {
    usbPrintf("\r\nBank A");
    gpioBank = GPIOA;
  } else if (iBank == 'B' | iBank == 'b')
    gpioBank = GPIOB;
  else {
    usbPrintf("Invalid Gpio bank number!\r\n");
    return -1;
  }
  usbPrintf("\r\nPlease input GPIO Pin(0-F):");
  usbRead(&cGpioPin, 1);
  usbPrintf(&cGpioPin);
  nGpioPin = char2int(cGpioPin);
  usbPrintf("\r\n");
  usbPrintf("\r\nPlease input GPIO Direction(0-In, 1-Out): ");
  usbRead(&gpioDir, 1);
  usbPrintf(&gpioDir);
  GPIO_InitStruct.Pin = ((uint16_t)0x0001 << nGpioPin);
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
 
  if (gpioDir == 0x31) {
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(gpioBank, &GPIO_InitStruct);
    usbPrintf("\r\nPlease input 1 or 0 to put gpio high or low: ");
    usbRead(&cGpioOutVal, 1);
    usbPrintf(&cGpioOutVal);
    nGpioOutVal = char2int(cGpioOutVal);
    usbPrintf("\r\n");
    
    HAL_GPIO_WritePin(gpioBank, GPIO_InitStruct.Pin, (GPIO_PinState)nGpioOutVal);

  } else {
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init(gpioBank, &GPIO_InitStruct);
    HAL_Delay(100);
    usbPrintf("\r\nThe gpio value is: %d\r\n", HAL_GPIO_ReadPin(gpioBank, GPIO_InitStruct.Pin));
    
  }  
  return 0;
}

void i2c_test(void)
{
  uint8_t data = 24;
  while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)I2C_ADDRESS, &data, 1, 500) != HAL_OK) {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
      usbPrintf("\r\nI2C1 access error!\r\n");
    }
  }
  while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)I2C_ADDRESS, &data, 1, 500) != HAL_OK) {
    if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
      usbPrintf("\r\nI2C1 access error!\r\n");
    }
  }
  usbPrintf("The value is: %x\r\n", data);
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

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00506682;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

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
