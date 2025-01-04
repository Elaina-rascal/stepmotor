/**
 * @file StepMotor.h
 * @author Elaina (1463967532@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-01-04
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H
#include "main.h"
#include "stdint.h"
#include "string.h"
#define BUFFER_SIZE 2000
/**
 * @brief 步进调用定时器默认的基频为1MHz
 * 分辨率为100
 *
 */
class StepMotor_t
{
public:
    // 给一定频率一定数量的脉冲
    StepMotor_t()
    {
    }
    StepMotor_t(TIM_HandleTypeDef *tim, uint32_t channel, GPIO_TypeDef *ph_port, uint16_t ph_pin)
    {
        _tim = tim;
        _ph_port = ph_port;
        _ph_pin = ph_pin;
        _channel = channel;
    }
    void givePulse(uint32_t pulse, uint32_t freq);
    void dmaCallBack(void);

private:
    TIM_HandleTypeDef *_tim;
    GPIO_TypeDef *_ph_port;
    uint32_t _channel;
    uint16_t _ph_pin;                         // 预分频
    uint32_t _clock_base_frequency = 1000000; // 时钟频率
    uint8_t _resolution = 100;                // 分辨率
    uint8_t _buffer[BUFFER_SIZE];
};

#endif