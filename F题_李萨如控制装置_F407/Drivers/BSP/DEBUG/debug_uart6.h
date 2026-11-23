#ifndef DEBUG_UART6_H
#define DEBUG_UART6_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

/* USART6: PC6 = TX, PC7 = RX. */
#define DEBUG_UART6_TX_PORT GPIOC
#define DEBUG_UART6_TX_PIN  GPIO_PIN_6
#define DEBUG_UART6_RX_PORT GPIOC
#define DEBUG_UART6_RX_PIN  GPIO_PIN_7

void DebugUart6_Init(uint32_t baudrate);
/** 开启 USART6 底层发送；VOFA 是否输出由 ADC 菜单单独控制。 */
void DebugUart6_SetOutputEnabled(uint8_t enabled);
HAL_StatusTypeDef DebugUart6_Write(const uint8_t *data, uint16_t length);
int DebugUart6_WriteBlocking(const uint8_t *data, uint16_t length);
/** Non-blocking single-byte receive from the USART6 RX ring. */
uint8_t DebugUart6_ReadByte(uint8_t *data);
/* FireWater: one channel name followed by a numeric CSV payload. */
int DebugUart6_VofaPrintf(const char *channel, const char *format, ...);
int DebugUart6_VofaPrintfBlocking(const char *channel, const char *format, ...);
/* FireWater helpers: one prefix followed by comma-separated numeric values. */
int DebugUart6_VofaPairPrintfBlocking(const char *channel1, float value1,
                                      const char *channel2, float value2);
int DebugUart6_VofaFivePrintfBlocking(const char *channel1, float value1,
                                      const char *channel2, float value2,
                                      const char *channel3, float value3,
                                      const char *channel4, float value4,
                                      const char *channel5, float value5);
/* Legacy text output is intentionally disabled to keep the VOFA stream clean. */
int DebugUart6_Printf(const char *format, ...);
int DebugUart6_PrintfBlocking(const char *format, ...);
/* Plain CRLF text for one-shot diagnostics; does not add a VOFA prefix. */
int DebugUart6_TextPrintfBlocking(const char *format, ...);
uint32_t DebugUart6_GetOverflowCount(void);
uint32_t DebugUart6_GetRxOverflowCount(void);
uint16_t DebugUart6_GetPendingBytes(void);

#endif /* DEBUG_UART6_H */
