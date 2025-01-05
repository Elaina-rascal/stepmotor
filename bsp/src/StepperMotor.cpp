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
        return;
        break;
    case Start:
        _state = WaitIteration;
        break;
    case WaitIteration:
        if (_iteration == 2 * _itertration_num)
        {
            _iteration = 0;
            _state = IDLE;
        }
        else
        {
            float temp_rpm;
            uint32_t temp_pulse;
            getIterationData(temp_rpm, temp_pulse);
            giveRPMPulse(temp_rpm, temp_pulse);
            _iteration++;
        }
        break;
    default:
        break;
    }
}
void StepMotor_t::giveRPMPulseSoft(float rpm, uint32_t pulse, uint16_t target_iteration_num, uint16_t itertion_pulse)
{
    if (target_iteration_num != 0 && itertion_pulse != 0)
    {
        _itertration_num = target_iteration_num;
        _iteration_pulse = itertion_pulse;
    }
    _soft_target_pulse = pulse;
    _soft_target_rpm = rpm;

    if (_state != IDLE || pulse == 0)
    {
        return;
    }
    // 算出剩余的脉冲数量
    else
    {
        _state = Start;
    }
}
void StepMotor_t::giveRPMPulse(float rpm, uint32_t pulse)
{
    uint32_t freq = _Subdivision * abs(rpm) * (360 / _StepAngle);
    if (pulse == 0)
    {
        return;
    }
    // 脉冲数量
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
    _target_pulse = BUFFER_SIZE / period > pulse ? pulse : BUFFER_SIZE / period;
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
void StepMotor_t::dmaCallBack(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != _tim->Instance)
    {
        return;
    }
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
        // _state = IDLE;
        _state = WaitIteration;
    }
}

uint32_t StepMotor_t::angleToPulse(float angle)
{
    return angle / _StepAngle * _Subdivision;
}

void StepMotor_t::getIterationData(float &rpm_in, uint32_t &pulse_in)
{
    if (_iteration < _itertration_num)
    {
        rpm_in = _soft_target_rpm * _iteration / _itertration_num;
        pulse_in = abs(rpm_in) * _iteration_pulse;
        _soft_target_pulse -= 2 * pulse_in;
        if (pulse_in < 0)
        {
            pulse_in = 0;
        }
    }
    else if (_iteration == _itertration_num)
    {
        rpm_in = _soft_target_rpm;
        pulse_in = _soft_target_pulse;
    }
    else if (_iteration <= 2 * _itertration_num)
    {
        rpm_in = _soft_target_rpm * (_itertration_num * 2 - _iteration) / _itertration_num;
        pulse_in = abs(rpm_in) * _iteration_pulse;
    }
    else
    {
        rpm_in = 0;
        pulse_in = 0;
    }
}