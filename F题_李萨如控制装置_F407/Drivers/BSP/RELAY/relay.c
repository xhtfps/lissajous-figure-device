/**
 * @file    relay.c
 * @brief   双路独立继电器驱动（AD9959 / DAC 两个 Y 源独立使能）。
 *
 * 原《简易相位差测量仪》OUT1 单刀双掷（二选一）已改为两路独立
 * 继电器：PG11 使能 AD9959、PG12 使能 DAC，均为高电平吸合。
 */

#include "relay.h"

static uint8_t s_dds_enabled;
static uint8_t s_dac_enabled;

/**
 * @brief  初始化继电器控制引脚。
 *         先把两脚预置为安全电平（全关、低电平），再切换为推挽输出，
 *         避免引脚从复位态输入变为输出时继电器瞬间误动作。
 */
void Relay_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOG_CLK_ENABLE();

    /* 先写电平，再改 MODER，防止过渡态产生短促继电器脉冲 */
    HAL_GPIO_WritePin(RELAY_DDS_GPIO_PORT, RELAY_DDS_CTRL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RELAY_DAC_GPIO_PORT, RELAY_DAC_CTRL_PIN, GPIO_PIN_RESET);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = RELAY_DDS_CTRL_PIN;
    HAL_GPIO_Init(RELAY_DDS_GPIO_PORT, &gpio);
    gpio.Pin = RELAY_DAC_CTRL_PIN;
    HAL_GPIO_Init(RELAY_DAC_GPIO_PORT, &gpio);

    s_dds_enabled = 0U;
    s_dac_enabled = 0U;
}

/**
 * @brief  独立设置 AD9959 继电器。
 * @param  enabled 非零 = 吸合（AD9959 接入 Y 轴），0 = 释放。
 */
void Relay_SetDDSEnabled(uint8_t enabled)
{
    s_dds_enabled = (enabled != 0U) ? 1U : 0U;
    HAL_GPIO_WritePin(RELAY_DDS_GPIO_PORT, RELAY_DDS_CTRL_PIN,
                      s_dds_enabled ? RELAY_DDS_ENABLE_LEVEL : GPIO_PIN_RESET);
}

/**
 * @brief  独立设置 DAC 继电器。
 * @param  enabled 非零 = 吸合（DAC 接入 Y 轴），0 = 释放。
 */
void Relay_SetDACEnabled(uint8_t enabled)
{
    s_dac_enabled = (enabled != 0U) ? 1U : 0U;
    HAL_GPIO_WritePin(RELAY_DAC_GPIO_PORT, RELAY_DAC_CTRL_PIN,
                      s_dac_enabled ? RELAY_DAC_ENABLE_LEVEL : GPIO_PIN_RESET);
}

void Relay_EnableDDS(void)  { Relay_SetDDSEnabled(1U); }
void Relay_DisableDDS(void) { Relay_SetDDSEnabled(0U); }
void Relay_EnableDAC(void)  { Relay_SetDACEnabled(1U); }
void Relay_DisableDAC(void) { Relay_SetDACEnabled(0U); }

uint8_t Relay_DDSEnabled(void) { return s_dds_enabled; }
uint8_t Relay_DACEnabled(void) { return s_dac_enabled; }
