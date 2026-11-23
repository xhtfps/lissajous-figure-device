#ifndef VISION_SLAVE_H
#define VISION_SLAVE_H

#include <stdint.h>

typedef enum
{
    VISION_SLAVE_COMMAND_NONE = 0U,
    VISION_SLAVE_COMMAND_SET,
    VISION_SLAVE_COMMAND_STOP,
    VISION_SLAVE_COMMAND_LOCKED,
    VISION_SLAVE_COMMAND_DONE,
    VISION_SLAVE_COMMAND_FAILED,
    VISION_SLAVE_COMMAND_RUN,
    VISION_SLAVE_COMMAND_RAMP
} VisionSlaveCommandKind_t;

typedef struct
{
    VisionSlaveCommandKind_t kind;
    uint16_t sequence;
    uint16_t probe_id;
    uint32_t frequency_millihz;
    uint32_t input_frequency_millihz;
    uint32_t phase_millideg;
    float amplitude_mv;
    uint8_t target_shape;
    uint8_t error_code;
    /* v3：$RAMP 斜坡探测参数（saw_hz=锯齿频率，amp_mv=斜坡幅度 mV） */
    uint32_t saw_hz;
} VisionSlaveCommand_t;

void VisionSlave_Init(void);
uint8_t VisionSlave_Poll(uint32_t now_ms,
                         VisionSlaveCommand_t *command);
/* 发送 $START,shape,prior_freq_mHz：prior 为断开前最后测得的输入频率
 * 先验（单位 mHz，0=无先验）。K230 v2 用它做局部扫频中心，避免 10s
 * 限时内对 1k~100kHz 全谱盲扫。 */
uint8_t VisionSlave_SendStart(uint8_t target_shape,
                              uint32_t prior_frequency_millihz);
uint8_t VisionSlave_SendApplied(uint16_t sequence,
                                uint16_t probe_id);
uint8_t VisionSlave_SendError(uint16_t sequence,
                              uint16_t probe_id,
                              uint8_t error_code);
uint8_t VisionSlave_IsOnline(uint32_t now_ms);

#endif /* VISION_SLAVE_H */
