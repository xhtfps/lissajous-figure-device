#include "debug_uart6.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define DEBUG_UART6_RING_SIZE 1024U
#define DEBUG_UART6_RING_MASK (DEBUG_UART6_RING_SIZE - 1U)
#define DEBUG_UART6_RX_RING_SIZE 256U
#define DEBUG_UART6_RX_RING_MASK (DEBUG_UART6_RX_RING_SIZE - 1U)
#define DEBUG_UART6_LINE_SIZE 320U

static UART_HandleTypeDef s_debug_uart6;
static uint8_t s_debug_uart6_ring[DEBUG_UART6_RING_SIZE];
static volatile uint16_t s_debug_uart6_head;
static volatile uint16_t s_debug_uart6_tail;
static volatile uint32_t s_debug_uart6_overflow;
static uint8_t s_debug_uart6_rx_ring[DEBUG_UART6_RX_RING_SIZE];
static volatile uint16_t s_debug_uart6_rx_head;
static volatile uint16_t s_debug_uart6_rx_tail;
static volatile uint32_t s_debug_uart6_rx_overflow;
static volatile uint8_t s_debug_uart6_ready;
static volatile uint8_t s_debug_uart6_output_enabled;
static char s_debug_uart6_line[DEBUG_UART6_LINE_SIZE];

static int DebugUart6_SendBlocking(const uint8_t *data, uint16_t length);

static uint32_t DebugUart6_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void DebugUart6_ExitCritical(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void DebugUart6_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef gpio_init;

    if (s_debug_uart6_ready != 0U)
    {
        return;
    }

    (void)memset(&gpio_init, 0, sizeof(gpio_init));
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART6_CLK_ENABLE();

    gpio_init.Pin = DEBUG_UART6_TX_PIN | DEBUG_UART6_RX_PIN;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    (void)memset(&s_debug_uart6, 0, sizeof(s_debug_uart6));
    s_debug_uart6.Instance = USART6;
    s_debug_uart6.Init.BaudRate = baudrate;
    s_debug_uart6.Init.WordLength = UART_WORDLENGTH_8B;
    s_debug_uart6.Init.StopBits = UART_STOPBITS_1;
    s_debug_uart6.Init.Parity = UART_PARITY_NONE;
    s_debug_uart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_debug_uart6.Init.Mode = UART_MODE_TX_RX;
    s_debug_uart6.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&s_debug_uart6) != HAL_OK)
    {
        return;
    }

    s_debug_uart6_head = 0U;
    s_debug_uart6_tail = 0U;
    s_debug_uart6_overflow = 0U;
    s_debug_uart6_rx_head = 0U;
    s_debug_uart6_rx_tail = 0U;
    s_debug_uart6_rx_overflow = 0U;
    s_debug_uart6_output_enabled = 0U;
    s_debug_uart6_ready = 1U;

    HAL_NVIC_SetPriority(USART6_IRQn, 7U, 0U);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    __HAL_UART_ENABLE_IT(&s_debug_uart6, UART_IT_RXNE);
}

void DebugUart6_SetOutputEnabled(uint8_t enabled)
{
    uint32_t primask;

    primask = DebugUart6_EnterCritical();
    s_debug_uart6_output_enabled = (enabled != 0U) ? 1U : 0U;
    DebugUart6_ExitCritical(primask);
}

HAL_StatusTypeDef DebugUart6_Write(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    uint16_t next_head;
    uint32_t primask;
    HAL_StatusTypeDef status = HAL_OK;

    if ((data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }
    if (s_debug_uart6_ready == 0U)
    {
        return HAL_ERROR;
    }
    if (s_debug_uart6_output_enabled == 0U)
    {
        return HAL_OK;
    }

    primask = DebugUart6_EnterCritical();
    for (index = 0U; index < length; ++index)
    {
        next_head = (uint16_t)((s_debug_uart6_head + 1U) &
                               DEBUG_UART6_RING_MASK);
        if (next_head == s_debug_uart6_tail)
        {
            s_debug_uart6_overflow += (uint32_t)(length - index);
            status = HAL_BUSY;
            break;
        }

        s_debug_uart6_ring[s_debug_uart6_head] = data[index];
        s_debug_uart6_head = next_head;
    }

    if (s_debug_uart6_head != s_debug_uart6_tail)
    {
        __HAL_UART_ENABLE_IT(&s_debug_uart6, UART_IT_TXE);
    }
    DebugUart6_ExitCritical(primask);
    return status;
}

int DebugUart6_WriteBlocking(const uint8_t *data, uint16_t length)
{
    return DebugUart6_SendBlocking(data, length);
}

uint8_t DebugUart6_ReadByte(uint8_t *data)
{
    uint32_t primask;

    if ((data == NULL) || (s_debug_uart6_ready == 0U))
    {
        return 0U;
    }

    primask = DebugUart6_EnterCritical();
    if (s_debug_uart6_rx_tail == s_debug_uart6_rx_head)
    {
        DebugUart6_ExitCritical(primask);
        return 0U;
    }

    *data = s_debug_uart6_rx_ring[s_debug_uart6_rx_tail];
    s_debug_uart6_rx_tail = (uint16_t)(
        (s_debug_uart6_rx_tail + 1U) & DEBUG_UART6_RX_RING_MASK);
    DebugUart6_ExitCritical(primask);
    return 1U;
}

/* Text frames are deliberately suppressed; FireWater is the only data path. */
int DebugUart6_Printf(const char *format, ...)
{
    (void)format;
    return 0;
}

int DebugUart6_PrintfBlocking(const char *format, ...)
{
    (void)format;
    return 0;
}

int DebugUart6_TextPrintfBlocking(const char *format, ...)
{
    va_list args;
    int payload_length;
    int total_length;

    if ((format == NULL) ||
        (s_debug_uart6_output_enabled == 0U))
    {
        return 0;
    }

    va_start(args, format);
    payload_length = vsnprintf(
        s_debug_uart6_line,
        sizeof(s_debug_uart6_line) - 2U,
        format, args);
    va_end(args);
    if ((payload_length < 0) ||
        (payload_length >= (int)sizeof(s_debug_uart6_line) - 2)
        )
    {
        return -1;
    }

    total_length = payload_length;
    s_debug_uart6_line[total_length] = '\r';
    s_debug_uart6_line[total_length + 1] = '\n';
    return DebugUart6_SendBlocking(
        (const uint8_t *)s_debug_uart6_line,
        (uint16_t)(total_length + 2));
}

static int DebugUart6_FormatVofaLine(const char *channel,
                                     const char *format,
                                     va_list args)
{
    int prefix_length;
    int payload_length;
    int total_length;

    if ((channel == NULL) || (format == NULL))
    {
        return -1;
    }

    prefix_length = snprintf(s_debug_uart6_line,
                             sizeof(s_debug_uart6_line), "%s:", channel);
    if ((prefix_length < 0) ||
        (prefix_length >= (int)sizeof(s_debug_uart6_line)))
    {
        return -1;
    }

    payload_length = vsnprintf(
        s_debug_uart6_line + prefix_length,
        sizeof(s_debug_uart6_line) - (size_t)prefix_length - 2U,
        format, args);
    if (payload_length < 0)
    {
        return -1;
    }

    total_length = prefix_length + payload_length;
    if (total_length >= (int)sizeof(s_debug_uart6_line) - 2)
    {
        return -1;
    }

    s_debug_uart6_line[total_length] = '\r';
    s_debug_uart6_line[total_length + 1] = '\n';
    s_debug_uart6_line[total_length + 2] = '\0';
    return total_length + 2;
}

static int DebugUart6_SendBlocking(const uint8_t *data, uint16_t length)
{
    uint16_t tail;

    if ((data == NULL) || (length == 0U) ||
        (s_debug_uart6_ready == 0U) ||
        (s_debug_uart6_output_enabled == 0U))
    {
        return 0;
    }

    __HAL_UART_DISABLE_IT(&s_debug_uart6, UART_IT_TXE);
    while (s_debug_uart6_tail != s_debug_uart6_head)
    {
        while ((__HAL_UART_GET_FLAG(&s_debug_uart6, UART_FLAG_TXE) == RESET))
        {
        }
        tail = s_debug_uart6_tail;
        s_debug_uart6.Instance->DR = s_debug_uart6_ring[tail];
        s_debug_uart6_tail = (uint16_t)((tail + 1U) &
                                        DEBUG_UART6_RING_MASK);
    }

    for (tail = 0U; tail < length; ++tail)
    {
        while ((__HAL_UART_GET_FLAG(&s_debug_uart6, UART_FLAG_TXE) == RESET))
        {
        }
        s_debug_uart6.Instance->DR = data[tail];
    }
    while ((__HAL_UART_GET_FLAG(&s_debug_uart6, UART_FLAG_TC) == RESET))
    {
    }
    if (s_debug_uart6_head != s_debug_uart6_tail)
    {
        __HAL_UART_ENABLE_IT(&s_debug_uart6, UART_IT_TXE);
    }
    return (int)length;
}

int DebugUart6_VofaPrintf(const char *channel, const char *format, ...)
{
    va_list args;
    int length;

    if (s_debug_uart6_output_enabled == 0U)
    {
        return 0;
    }
    va_start(args, format);
    length = DebugUart6_FormatVofaLine(channel, format, args);
    va_end(args);
    if (length <= 0)
    {
        return length;
    }
    return (DebugUart6_Write((const uint8_t *)s_debug_uart6_line,
                             (uint16_t)length) == HAL_OK) ? length : -1;
}

int DebugUart6_VofaPrintfBlocking(const char *channel, const char *format, ...)
{
    va_list args;
    int length;

    if (s_debug_uart6_output_enabled == 0U)
    {
        return 0;
    }
    va_start(args, format);
    length = DebugUart6_FormatVofaLine(channel, format, args);
    va_end(args);
    if (length <= 0)
    {
        return length;
    }
    return DebugUart6_SendBlocking((const uint8_t *)s_debug_uart6_line,
                                   (uint16_t)length);
}

int DebugUart6_VofaPairPrintfBlocking(const char *channel1, float value1,
                                      const char *channel2, float value2)
{
    int length;

    if ((channel1 == NULL) || (channel2 == NULL) ||
        (s_debug_uart6_output_enabled == 0U))
    {
        return 0;
    }

    (void)channel2;
    length = snprintf(s_debug_uart6_line, sizeof(s_debug_uart6_line),
                      "%s:%.3f,%.3f\r\n",
                      channel1, (double)value1,
                      (double)value2);
    if ((length <= 0) || (length >= (int)sizeof(s_debug_uart6_line)))
    {
        return -1;
    }

    return DebugUart6_SendBlocking((const uint8_t *)s_debug_uart6_line,
                                   (uint16_t)length);
}

int DebugUart6_VofaFivePrintfBlocking(const char *channel1, float value1,
                                      const char *channel2, float value2,
                                      const char *channel3, float value3,
                                      const char *channel4, float value4,
                                      const char *channel5, float value5)
{
    int length;

    if ((channel1 == NULL) || (channel2 == NULL) || (channel3 == NULL) ||
        (channel4 == NULL) || (channel5 == NULL) ||
        (s_debug_uart6_output_enabled == 0U))
    {
        return 0;
    }

    (void)channel2;
    (void)channel3;
    (void)channel4;
    (void)channel5;
    length = snprintf(
        s_debug_uart6_line, sizeof(s_debug_uart6_line),
        "%s:%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
        channel1, (double)value1, (double)value2,
        (double)value3, (double)value4, (double)value5);
    if ((length <= 0) || (length >= (int)sizeof(s_debug_uart6_line)))
    {
        return -1;
    }

    return DebugUart6_SendBlocking((const uint8_t *)s_debug_uart6_line,
                                   (uint16_t)length);
}

uint32_t DebugUart6_GetOverflowCount(void)
{
    uint32_t primask;
    uint32_t overflow;

    primask = DebugUart6_EnterCritical();
    overflow = s_debug_uart6_overflow;
    DebugUart6_ExitCritical(primask);
    return overflow;
}

uint32_t DebugUart6_GetRxOverflowCount(void)
{
    uint32_t primask;
    uint32_t overflow;

    primask = DebugUart6_EnterCritical();
    overflow = s_debug_uart6_rx_overflow;
    DebugUart6_ExitCritical(primask);
    return overflow;
}

uint16_t DebugUart6_GetPendingBytes(void)
{
    uint32_t primask;
    uint16_t head;
    uint16_t tail;
    uint16_t pending;

    primask = DebugUart6_EnterCritical();
    head = s_debug_uart6_head;
    tail = s_debug_uart6_tail;
    pending = (head >= tail) ? (uint16_t)(head - tail) :
              (uint16_t)(DEBUG_UART6_RING_SIZE - tail + head);
    DebugUart6_ExitCritical(primask);
    return pending;
}

void USART6_IRQHandler(void)
{
    if ((__HAL_UART_GET_FLAG(&s_debug_uart6, UART_FLAG_RXNE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&s_debug_uart6, UART_IT_RXNE) != RESET))
    {
        uint8_t data = (uint8_t)s_debug_uart6.Instance->DR;
        uint16_t next_head = (uint16_t)(
            (s_debug_uart6_rx_head + 1U) & DEBUG_UART6_RX_RING_MASK);

        if (next_head == s_debug_uart6_rx_tail)
        {
            ++s_debug_uart6_rx_overflow;
        }
        else
        {
            s_debug_uart6_rx_ring[s_debug_uart6_rx_head] = data;
            s_debug_uart6_rx_head = next_head;
        }
    }

    if (__HAL_UART_GET_FLAG(&s_debug_uart6, UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(&s_debug_uart6);
        ++s_debug_uart6_rx_overflow;
    }

    if ((__HAL_UART_GET_FLAG(&s_debug_uart6, UART_FLAG_TXE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&s_debug_uart6, UART_IT_TXE) != RESET))
    {
        if (s_debug_uart6_tail != s_debug_uart6_head)
        {
            s_debug_uart6.Instance->DR =
                s_debug_uart6_ring[s_debug_uart6_tail];
            s_debug_uart6_tail = (uint16_t)((s_debug_uart6_tail + 1U) &
                                            DEBUG_UART6_RING_MASK);
        }
        else
        {
            __HAL_UART_DISABLE_IT(&s_debug_uart6, UART_IT_TXE);
        }
    }
}
