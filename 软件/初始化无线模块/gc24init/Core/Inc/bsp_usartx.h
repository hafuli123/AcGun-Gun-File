#ifndef __BSP_USARTX_H__
#define __BSP_USARTX_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/

//����ǹ
#define USARTx                                 USART2
#define USARTx_BAUDRATE                        9600
#define USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART2_CLK_ENABLE()
#define USART_RCC_CLK_DISABLE()                __HAL_RCC_USART2_CLK_DISABLE()

#define USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_Tx_GPIO_PIN                     GPIO_PIN_2
#define USARTx_Tx_GPIO                         GPIOA
#define USARTx_Rx_GPIO_PIN                     GPIO_PIN_3   
#define USARTx_Rx_GPIO                         GPIOA

#define USARTx_AFx                             GPIO_AF7_USART2

#define USARTx_IRQHANDLER                      USART2_IRQHandler
#define USARTx_IRQn                            USART2_IRQn


/* ��չ���� ------------------------------------------------------------------*/
extern UART_HandleTypeDef husartx;

/* �������� ------------------------------------------------------------------*/
void MX_USARTx_Init(void);


#endif  /* __BSP_USARTX_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
