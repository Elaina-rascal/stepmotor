/**
 * @file StepMotor.h
 * @author Elaina (1463967532@qq.com)
 * @brief 使用方法:
 * 1.先初始化一个步进电机实例,并且设置好参数
 * Motor = StepMotor_t(&htim2, TIM_CHANNEL_2, GPIOB, GPIO_PIN_4);
 * Motor.setParam(1.8, 8, 10, 100);
 * 2.注册回调函数
 * void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
 * {Motor.dmaCallBack();}
 * 3.在定时器中断,主循环或者rtos任务重复调用update函数,下面是主循环的例子
 * while(1)
 * {
 * Motor.update(1);
 * Hal_Delay(5);
 * }
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
#define BUFFER_SIZE 500  // 缓冲区大小越大占用cpu时间越少
#define ITERATIONS 10    // 缓起缓停迭代次数
#define ITERATIONPULSE 8 // 缓起缓停迭代角度
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
    /**
     * @brief 新建一个步进电机实例
     *
     * @param tim pwm对应的定时器
     * @param channel pwm对应的通道
     * @param ph_port 步进电机的相位端口
     * @param ph_pin 步进电机的相位引脚
     */
    StepMotor_t(TIM_HandleTypeDef *tim, uint32_t channel, GPIO_TypeDef *ph_port, uint16_t ph_pin)
    {
        _tim = tim;
        _ph_port = ph_port;
        _ph_pin = ph_pin;
        _channel = channel;
        __HAL_TIM_SetAutoreload(_tim, _resolution - 1);
    }
    /**
     * @brief 设置步进电机的参数
     *
     * @param StepAngle 步进角
     * @param Subdivision 细分
     * @param target_iteration_num 缓起迭代次数
     * @param itertion_pulse 缓起迭代脉冲数
     */
    void setParam(float StepAngle, uint8_t Subdivision, uint16_t target_iteration_num, uint16_t itertion_pulse)
    {
        _StepAngle = StepAngle;
        _Subdivision = Subdivision;
        _itertration_num = target_iteration_num;
        _iteration_pulse = itertion_pulse;
    }
    /**
     * @brief 是否空闲
     *
     * @return true
     * @return false
     */
    bool isBusy(void)
    {
        return !(_state == IDLE);
    }
    void update(uint16_t dt);
    /**
     * @brief 缓启缓停转动一定脉冲数
     *
     * @param rpm 转速
     * @param pulse 脉冲数
     * @param target_iteration_num 缓起迭代次数
     * @param itertion_pulse 缓起迭代脉冲数 越大迭代时间越长
     */
    void giveRPMPulseSoft(float rpm, uint32_t pulse, uint16_t target_iteration_num = 0, uint16_t itertion_pulse = 0);
    /**
     * @brief 按照一定的转速给一定数量的脉冲
     *
     * @param rpm 转数
     * @param pulse 脉冲数
     */
    void giveRPMPulse(float rpm, uint32_t pulse);
    /**
     * @brief 按照一定转速转一个角度
     *
     * @param rpm 转速
     * @param angle 角度
     */
    void giveRPMAngle(float rpm, float angle);
    /**
     * @brief dma回调
     *
     */
    void dmaCallBack(TIM_HandleTypeDef *htim);
    /**
     * @brief 将角度转换为脉冲数
     *
     * @param angle 角度
     * @return uint32_t 脉冲数
     */
    uint32_t angleToPulse(float angle);

private:
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
    /**
     * @brief Get the Iteration Data object 获取迭代数据
     *
     * @param rpm_in 迭代的转速
     * @param pulse_in 迭代的脉冲数
     */
    void getIterationData(float &rpm_in, uint32_t &pulse_in);
    /*缓启缓停相关*/
    uint16_t _iteration = 0;                    // 当前迭代次数
    uint32_t _soft_target_pulse = 0;            // 缓启缓停的目标脉冲数
    float _soft_target_rpm = 0;                 // 缓启缓停的目标最大转速
    uint16_t _itertration_num = ITERATIONS;     // 缓启缓停的迭代次数
    uint16_t _iteration_pulse = ITERATIONPULSE; // 缓启缓停的每圈转速对应的脉冲数
    /*步进电机参数相关*/
    float _StepAngle = 1.8;   // 步进角
    uint8_t _Subdivision = 8; // 细分
    /*给一定数量脉冲实现相关*/
    uint16_t _pulse_mod;           // 取余的脉冲数
    uint32_t _target_pulse = 0;    // 给一串脉冲的目标脉冲数
    uint32_t _target_freq = 20000; // 给一串脉冲的目标频率
    uint16_t _target_number = 0;   // 给一串脉冲的目标次数
    StepMotorState _state = IDLE;
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