# 双通道继电器切换驱动（AD9959 / DAC）使用说明

## 1. 来源

原工程 `E:\STM32_f4_project\往年题目\简易相位差测量仪` 中使用继电器做信号路由
切换，代码位于：

- `Drivers\BSP\PHASE\phase_route.c` / `phase_route.h`

该驱动用 GPIOG 的 PG9~PG15 共 7 路继电器控制 OUT1~OUT5 等多路信号路由
（其中 OUT1 就是典型的双通道切换：低电平选 IN1、高电平选 IN2）。

本工程按“只切换两个通道”的需求，精简为 **1 路继电器、2 通道**，新模块：

- `Drivers\BSP\RELAY\relay.c` / `relay.h`

## 2. 硬件约定（接线）

| 对象 | 引脚/接口 | 说明 |
|---|---|---|
| 控制脚 | **PG9** | F407 工程当前空闲，与 LCD/FSMC 无冲突（LCD 用 NE1=PD7） |
| 继电器模块 | **低电平触发** | IN 脚低电平 → 吸合（NO 与 COM 通）；高电平 → 释放（NC 与 COM 通） |
| COM | 示波器 Y 轴输入 | 公共端 |
| NO（常开） | AD9959 信号输出 | 低电平（吸合）时选通 |
| NC（常闭） | DAC 信号输出 | 高电平（释放）时选通 |

即：**PG9 = 低 → Y 轴接 AD9959；PG9 = 高 → Y 轴接 DAC。**

> 若你的继电器板高低电平含义相反（高电平触发），只改 `relay.h` 中
> `RELAY_DDS_LEVEL` / `RELAY_DAC_LEVEL` 两个宏即可，代码其余部分不用动。

## 3. API

```c
#include "./BSP/RELAY/relay.h"

Relay_Init();                              /* 上电初始化，默认切到 AD9959（PG9=低） */

Relay_SwitchChannel(RELAY_CH_DDS);         /* 切到 AD9959 信号（PG9 输出低电平） */
Relay_SwitchChannel(RELAY_CH_DAC);         /* 切到 DAC 信号（PG9 输出高电平）   */

RelayChannel_t ch = Relay_GetChannel();    /* 查询当前通道 */
```

- `Relay_Init()` 已在 `User\main.c` 板级初始化中调用（`led_init()` 之后），
  上电默认选中 AD9959 通道，无需重复调用。
- 切换函数在任意任务/中断上下文都能调用，只需一条语句。

## 4. 使用示例（切到 DAC 通道后切回）

```c
/* 例如：要求 1 直通时用 DAC 信号，测量/控制时用 AD9959 信号 */
Relay_SwitchChannel(RELAY_CH_DAC);          /* 示波器 Y 轴改接 DAC 输出 */
delay_ms(10);                               /* 继电器动作 + 模拟通路稳定 */
Relay_SwitchChannel(RELAY_CH_DDS);          /* 切回 AD9959 输出 */
```

## 5. 移植清单

| 动作 | 文件 | 状态 |
|---|---|---|
| 新建驱动 | `Drivers\BSP\RELAY\relay.c` + `relay.h` | 已创建 |
| 加入 Keil 工程 | `Projects\MDK-ARM\atk_f407.uvprojx`（PHASE 组后新增 relay.c） | 已添加 |
| 板级初始化 | `User\main.c`（include + `Relay_Init()` 调用） | 已接入 |
| 编译验证 | Keil UV4 命令行构建 | **0 Error / 0 Warning** |

## 6. 引脚冲突检查结论

- PG10 已被 `f_app.c` 用作完成灯输出（GPIOG 普通 GPIO 用法可行）。
- LCD 的 FSMC 初始化只配置 GPIOD/GPIOE/GPIOF0，未触碰 GPIOG；PG9 复位态为
  普通输入，`Relay_Init()` 将其切为推挽输出，无冲突。
- 若后续要外扩 SRAM 或启用 FSMC Bank1 的 NE2/NE3/NE4，需重新评估 PG9/PG10/PG12。
