#include <stdint.h>
#include "stm8l15x.h"
#include "gpio.h"
#include "uart-board.h"

#define UART_BUF_SIZE  640
//CONFIG_UART_DEBUG=0 //默认 现在已经更改
//CONFIG_UART_CMD=1  //默认现在已更改


#define COMn                        2
#define EVAL_COM0                   USART3
#define EVAL_COM0_GPIO              GPIOG
#define EVAL_COM0_CLK               CLK_Peripheral_USART3
#define EVAL_COM0_RxPin             GPIO_Pin_0
#define EVAL_COM0_TxPin             GPIO_Pin_1

#define EVAL_COM1                   USART2
#define EVAL_COM1_GPIO              GPIOH
#define EVAL_COM1_CLK               CLK_Peripheral_USART2
#define EVAL_COM1_RxPin             GPIO_Pin_4
#define EVAL_COM1_TxPin             GPIO_Pin_5

#if CONFIG_UART_CMD == 0
#define SCGW_COM                    EVAL_COM0
#elif CONFIG_UART_CMD == 1
#define SCGW_COM                    EVAL_COM1
#else
#error "no scgw command uart selected"
#endif

#if CONFIG_UART_DEBUG == 0
#define PRINTF_COM                    EVAL_COM0
#elif CONFIG_UART_DEBUG == 1
#define PRINTF_COM                    EVAL_COM1
#else
#error "no debug uart selected"
#endif

#ifdef _RAISONANCE_
#define PUTCHAR_PROTOTYPE int putchar (char c)
#define GETCHAR_PROTOTYPE int getchar (void)
#elif defined (_COSMIC_)
#define PUTCHAR_PROTOTYPE char putchar (char c)
#define GETCHAR_PROTOTYPE char getchar (void)
#else /* _IAR_ */
#define PUTCHAR_PROTOTYPE int putchar (int c)
#define GETCHAR_PROTOTYPE int getchar (void)
#endif /* _RAISONANCE_ */


static const USART_TypeDef* COM_USART[COMn] =
  {
    EVAL_COM0,
    EVAL_COM1
  };

static const GPIO_TypeDef* COM_PORT[COMn] =
  {
    EVAL_COM0_GPIO,
    EVAL_COM1_GPIO
  };
static const CLK_Peripheral_TypeDef COM_USART_CLK[COMn] =
  {
    EVAL_COM0_CLK,
    EVAL_COM1_CLK
  };
static const uint8_t COM_TX_PIN[COMn] =
  {
    EVAL_COM0_TxPin,
    EVAL_COM1_TxPin
  };
static const uint8_t COM_RX_PIN[COMn] =
  {
    EVAL_COM0_RxPin,
    EVAL_COM1_RxPin
  };



void COMInit(COM_TypeDef COM, uint32_t USART_BaudRate,
                      USART_WordLength_TypeDef USART_WordLength,
                      USART_StopBits_TypeDef USART_StopBits,
                      USART_Parity_TypeDef USART_Parity,
                      USART_Mode_TypeDef USART_Mode)
{
    CLK_PeripheralClockConfig(COM_USART_CLK[COM], ENABLE);
    GPIO_ExternalPullUpConfig((GPIO_TypeDef *)COM_PORT[COM], COM_TX_PIN[COM], ENABLE);
    GPIO_ExternalPullUpConfig((GPIO_TypeDef *)COM_PORT[COM], COM_RX_PIN[COM], ENABLE);
    USART_Init((USART_TypeDef *)COM_USART[COM], USART_BaudRate,
         USART_WordLength,
         USART_StopBits,
         USART_Parity,
         USART_Mode);
}

void COMDeInit(COM_TypeDef COM)
{
    USART_Cmd((USART_TypeDef *)COM_USART[COM], false);
    USART_DeInit((USART_TypeDef *)COM_USART[COM]);
    CLK_PeripheralClockConfig(COM_USART_CLK[COM], DISABLE);    
}

/**
  * @brief Retargets the C library printf function to the USART.
  * @param[in] c Character to send
  * @retval char Character sent
  * @par Required preconditions:
  * - None
  */
PUTCHAR_PROTOTYPE
{
  /* Write a character to the USART */
  USART_SendData8(PRINTF_COM, c);
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(PRINTF_COM, USART_FLAG_TC) == RESET);

  return (c);
}
/**
  * @brief Retargets the C library scanf function to the USART.
  * @param[in] None
  * @retval char Character to Read
  * @par Required preconditions:
  * - None
  */
GETCHAR_PROTOTYPE
{
  int c = 0;
  /* Loop until the Read data register flag is SET */
  while (USART_GetFlagStatus(PRINTF_COM, USART_FLAG_RXNE) == RESET);
    c = USART_ReceiveData8(PRINTF_COM);
    return (c);
}

void scgw_uart_write_char(char c)
{
  /* Write a character to the USART */
  USART_SendData8(SCGW_COM, c);
  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(SCGW_COM, USART_FLAG_TC) == RESET);
}

bool scgw_uart_read_char(char * c)
{
    if(USART_GetFlagStatus(SCGW_COM, USART_FLAG_RXNE) == RESET)
    {
        return false;
    }
    else
    {
        SCGW_COM->SR = (uint8_t)((uint16_t)~((uint16_t)USART_FLAG_RXNE));
        *c = USART_ReceiveData8(SCGW_COM);
        return true;      
    }
}

void USARTInit(void)
{
#if (CONFIG_UART_CMD == 0 || CONFIG_UART_DEBUG == 0)
    COMInit(COM0, (uint32_t)115200, USART_WordLength_8b, USART_StopBits_1,
                       USART_Parity_No, (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));   
#endif

#if (CONFIG_UART_CMD == 1 || CONFIG_UART_DEBUG == 1)    
    COMInit(COM1, (uint32_t)115200, USART_WordLength_8b, USART_StopBits_1,
                       USART_Parity_No, (USART_Mode_TypeDef)(USART_Mode_Tx | USART_Mode_Rx));   
    	USART_ITConfig (EVAL_COM1,USART_IT_OR,ENABLE);//使能接收中断,中断向量号为20
    USART_Cmd (EVAL_COM1,ENABLE);//使能USART
#endif    
}
