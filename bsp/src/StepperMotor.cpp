/**
 * @file StepperMotor.cpp
 * @author Elaina (1463967532@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-01-04
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "StepMotor.h"
void StepMotor_t::giveOncePulse(uint32_t pulse, uint32_t freq, bool clearbuffer)
{
    // 先算出每隔几个频率发一次脉冲
    uint32_t period = _clock_base_frequency / freq / _resolution;
    uint8_t arr = _resolution / 2;
    // 先清零缓冲区
    uint16_t max_address = pulse * period;
    if (max_address > BUFFER_SIZE)
    {
        return;
    }
    if (clearbuffer)
    {
        memset(_buffer, 0, BUFFER_SIZE);
        for (uint16_t i = 0; i < max_address; i += period)
        {
            _buffer[i] = arr;
        }
    }

    HAL_TIM_PWM_Start_DMA(_tim, _channel, (uint32_t *)_buffer, max_address);
}
void StepMotor_t::dmaCallBack(void)
{
    HAL_TIM_PWM_Stop_DMA(_tim, _channel);
}