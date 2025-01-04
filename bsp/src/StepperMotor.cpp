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
void StepMotor_t::givePulse(uint32_t pulse, uint32_t freq)
{
    // 先算出每隔几个频率发一次脉冲
    uint32_t period = _clock_base_frequency / freq / _resolution;
    uint8_t arr = _resolution / 2; // 占空比为50%
    // 先清零缓冲区
    uint16_t max_address = pulse * period;
    // 如果不被整除情况下要加一次
    _target_number = max_address / BUFFER_SIZE + (((max_address % BUFFER_SIZE) == 0) ? 0 : 1);
    // 每次循环的目标脉冲数,除了最后一次
    _target_pulse = pulse / _target_number;
    // 最后一次的脉冲数,考虑只有执行一次的情况
    _pulse_mod = pulse - _target_pulse * (_target_number - 1);
    if (_target_number == 1)
    {
        _pulse_mod = 0;
    }
    _target_freq = freq;
    giveOncePulse(_target_pulse, _target_freq);
}
void StepMotor_t::giveOncePulse(uint32_t pulse, uint32_t freq, bool clearbuffer)
{
    // 先算出每隔几个频率发一次脉冲
    uint32_t period = _clock_base_frequency / freq / _resolution;
    uint8_t arr = _resolution / 2; // 占空比为50%
    // 先清零缓冲区
    uint16_t max_address = pulse * period;
    if (max_address > BUFFER_SIZE)
    {
        return;
    }
    if (clearbuffer)
    {
        memset(_buffer, 0, BUFFER_SIZE);
        for (uint16_t i = 0; i < pulse; i += 1)
        {
            uint16_t j = i * period;
            _buffer[j] = arr;
        }
    }

    HAL_TIM_PWM_Start_DMA(_tim, _channel, (uint32_t *)_buffer, max_address);
}
void StepMotor_t::dmaCallBack(void)
{
    if (_target_number > 1)
    {
        giveOncePulse(_target_pulse, _target_freq, false);
        _target_number--;
    }
    else if (_target_number == 1 && _pulse_mod > 0)
    {
        giveOncePulse(_pulse_mod, _target_freq);
        _target_number--;
    }
    else
    {
        HAL_TIM_PWM_Stop_DMA(_tim, _channel);
    }
}