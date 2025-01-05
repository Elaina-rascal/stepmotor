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
#define BUFFER_SIZE 500
#define ITERATIONS 5     // 缓起缓停迭代次数
#define ITERATIONANGLE 4 // 缓起缓停迭代角度
enum StepMotorState
{
    BUSY,          // 正在输出脉冲
    IDLE,          // 空闲
    WaitIteration, // 等待迭代
    Start,         // 启动
};
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
        __HAL_TIM_SetAutoreload(_tim, _resolution - 1);
    }
    bool isBusy(void)
    {
        return !(_state == IDLE);
    }
    void update(uint16_t dt);
    void giveRPMAngle(float rpm, float angle, bool use_soft_start);
    void giveRPMAngle(float rpm, float angle);
    /**
     * @brief 给一定频率一定数量的脉冲,底层通过多次调用giveOncePulse来实现,通过回调来执行下一次调用
     *
     * @param pulse 脉冲数量
     * @param freq 脉冲的频率
     */
    void givePulse(uint32_t pulse, uint32_t freq = 20000);
    /**
     * @brief 给一串脉冲,脉冲的频率通过基频的间隔来实现
     *
     * @param pulse 脉冲数量
     * @param freq 脉冲的基频默认为20KHz
     * @param clear_buffer 是否清空缓冲区
     */
    void giveOncePulse(uint32_t pulse, uint32_t freq = 20000, bool clear_buffer = true);
    void dmaCallBack(void);

private:
    /*步进电机参数相关*/
    float _StepAngle = 1.8;   // 步进角
    uint8_t _Subdivision = 8; // 细分
    /*给一定数量脉冲实现相关*/
    uint16_t _pulse_mod;           // 取余的脉冲数
    uint32_t _target_pulse = 0;    // 给一串脉冲的目标脉冲数
    uint32_t _target_freq = 20000; // 给一串脉冲的目标频率
    uint16_t _target_number = 0;   // 给一串脉冲的目标次数
    StepMotorState _state = IDLE;
    float _iteration_state[4 * (ITERATIONS) + 2] = {0}; // 迭代状态
    uint16_t _iteration = 2 * ITERATIONS + 1;           // 迭代次数
    /*硬件外设相关*/
    TIM_HandleTypeDef *_tim;
    GPIO_TypeDef *_ph_port;
    uint32_t _channel;
    uint16_t _ph_pin;                         // 预分频
    uint32_t _clock_base_frequency = 1000000; // 时钟频率经过预分频后的频率
    uint8_t _resolution = 10;                 // 分辨率
    uint8_t _buffer[BUFFER_SIZE];
};

#endif