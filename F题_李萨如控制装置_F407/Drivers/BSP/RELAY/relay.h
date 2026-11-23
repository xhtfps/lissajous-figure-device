#ifndef __RELAY_H
#define __RELAY_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/*
 * 双路独立继电器驱动（AD9959 / DAC 两个 Y 源独立使能）
 *
 * 原《简易相位差测量仪》OUT1 单刀双掷（PG9 一高一低二选一）已改为
 * 两路独立继电器，每路一个 GPIO 独立控制，可单独开/关：
 *   - 引脚1 = PG11 -> 继电器1（IN1）-> AD9959 信号输出（DDS）
 *   - 引脚2 = PG12 -> 继电器2（IN2）-> DAC 信号输出（PA4 锯齿/正弦）
 *
 * 电平约定（均为高电平使能）：
 *   - 引脚输出高电平 -> 继电器吸合 -> 对应信号接入示波器 Y 轴
 *   - 引脚输出低电平 -> 继电器释放 -> 对应信号断开
 *   - 上电默认两路全关（低电平），避免复位过渡态误动作
 *
 * 推荐接线（两路 COM 并联到示波器 Y 轴输入）：
 *         继电器1 COM  <- 示波器 Y； 继电器1 NO  <- AD9959 信号
 *         继电器2 COM  <- 示波器 Y； 继电器2 NO  <- DAC 信号
 *
 * 若你的继电器模块是低电平触发，把 RELAY_DDS_ENABLE_LEVEL /
 * RELAY_DAC_ENABLE_LEVEL 两个宏改为 GPIO_PIN_RESET 即可，其余不用改。
 */
#define RELAY_DDS_GPIO_PORT        GPIOG
#define RELAY_DDS_CTRL_PIN         GPIO_PIN_11  /* 引脚1：AD9959 使能 */
#define RELAY_DAC_GPIO_PORT        GPIOG
#define RELAY_DAC_CTRL_PIN         GPIO_PIN_12  /* 引脚2：DAC 使能 */

#define RELAY_DDS_ENABLE_LEVEL     GPIO_PIN_SET /* 高电平 = 使能 AD9959 */
#define RELAY_DAC_ENABLE_LEVEL     GPIO_PIN_SET /* 高电平 = 使能 DAC */

/* 初始化 PG11/PG12 为推挽输出，默认两路全关（低电平）。 */
void Relay_Init(void);

/* AD9959 继电器独立使能/断开 */
void Relay_EnableDDS(void);
void Relay_DisableDDS(void);
void Relay_SetDDSEnabled(uint8_t enabled);

/* DAC 继电器独立使能/断开 */
void Relay_EnableDAC(void);
void Relay_DisableDAC(void);
void Relay_SetDACEnabled(uint8_t enabled);

/* 查询当前两路状态（1=使能/吸合，0=断开） */
uint8_t Relay_DDSEnabled(void);
uint8_t Relay_DACEnabled(void);

#endif /* __RELAY_H */
