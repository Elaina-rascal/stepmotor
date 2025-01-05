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
#define abs(x) ((x) > 0 ? (x) : -(x))
void StepMotor_t::update(uint16_t dt)
{
    // 1ms的时间间隔
    switch (_state)
    {
    case BUSY:
        return;
        break;
    case IDLE:
        if (_iteration == 2 * ITERATIONS + 1)
        {
            return;
        }
        else
        {
            giveRPMAngle(_iteration_state[_iteration * 2], _iteration_state[_iteration * 2 + 1]);
            _iteration++;
        }
        break;
    case Handle:
        _iteration = 0;
        _state = IDLE;
        break;
    default:
        break;
    }
}
void StepMotor_t::giveRPMAngle(float rpm, float angle, bool use_soft_start)
{
    if (!use_soft_start)
    {
        giveRPMAngle(rpm, angle);
    }
    else
    {
        float angle_sum = angle;
        // 先算出缓起缓停的脉冲数量
        for (int i = 0; i < ITERATIONS; i++)
        {
            _iteration_state[2 * i] = rpm / ITERATIONS * (i + 1);
            _iteration_state[2 * i + 1] = _iteration_state[i] * _StepAngle * ITERATIONANGLE;
            angle_sum -= _iteration_state[i * 2 + 1] * 2;
            _iteration_state[ITERATIONS * 4 - i * 2] = _iteration_state[2 * i];
            _iteration_state[ITERATIONS * 4 - i * 2 + 1] = _iteration_state[2 * i + 1];
        }
        _iteration_state[ITERATIONS * 2] = rpm;
        _iteration_state[ITERATIONS * 2 + 1] = angle_sum;
        if (angle_sum <= 0)
        {
            return;
        }
        // 算出剩余的角度
        else
        {
            _state = Handle;
        }
    }
}
void StepMotor_t::giveRPMAngle(float rpm, float angle)
{
    // 脉冲频率
    uint32_t freq = _Subdivision * abs(rpm) * (360 / _StepAngle);
    // 脉冲数量
    uint32_t pulse = _Subdivision * angle / _StepAngle;
    if (rpm > 0)
    {
        HAL_GPIO_WritePin(_ph_port, _ph_pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(_ph_port, _ph_pin, GPIO_PIN_RESET);
    }
    givePulse(pulse, freq);
}
void StepMotor_t::givePulse(uint32_t pulse, uint32_t freq)
{
    // 先算出每隔几个频率发一次脉冲
    uint32_t period = _clock_base_frequency / freq / _resolution;
    uint8_t arr = _resolution / 2; // 占空比为50%
    //  先清零缓冲区
    uint32_t max_address = pulse * period;
    // 如果不被整除情况下要加一次
    // _target_number = max_address / BUFFER_SIZE + (((max_address % BUFFER_SIZE) == 0) ? 0 : 1);
    // 每次循环的目标脉冲数,除了最后一次
    _target_pulse = BUFFER_SIZE / period;
    // 如果不被整除情况下要加一次
    _target_number = pulse / _target_pulse + ((pulse % _target_pulse == 0) ? 0 : 1);
    // 最后一次的脉冲数,考虑只有执行一次的情况
    _pulse_mod = pulse - _target_pulse * (_target_number - 1);
    if (_target_number == 1)
    {
        _pulse_mod = 0;
    }
    _target_freq = freq;
    giveOncePulse(_target_pulse, _target_freq);
    _state = BUSY;
}
void StepMotor_t::giveOncePulse(uint32_t pulse, uint32_t freq, bool clearbuffer)
{
    // 先算出每隔几个频率发一次脉冲
    uint32_t period = _clock_base_frequency / freq / _resolution;
    uint8_t ccr = _resolution / 2; // 占空比为50%
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
            _buffer[j] = ccr;
        }
    }

    HAL_TIM_PWM_Start_DMA(_tim, _channel, (uint32_t *)_buffer, max_address);
}
void StepMotor_t::dmaCallBack(void)
{
    if (_target_number > 2)
    {
        giveOncePulse(_target_pulse, _target_freq, false);
        _target_number--;
    }
    else if (_target_number == 2 && _pulse_mod > 0)
    {
        giveOncePulse(_pulse_mod, _target_freq);
        _target_number--;
    }
    else
    {
        HAL_TIM_PWM_Stop_DMA(_tim, _channel);
        _state = IDLE;
    }
}