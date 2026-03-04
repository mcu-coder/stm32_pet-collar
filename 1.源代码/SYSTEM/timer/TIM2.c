#include "TIM2.h"
#include "delay.h"
#include "stm32f10x.h"
#include "Key.h"
// TIM2中断计数器，用于测试延时准确性
static volatile uint32_t tim2_interrupt_count = 0;

/**
 * @brief  TIM2初始化（1ms定时中断）
 * @param  Prescaler: 预分频值
 * @param  Period: 自动重装载值
 * @note   公式：中断周期 = (Prescaler+1)*(Period+1)/系统时钟频率(Hz)
 *          系统时钟72MHz时，要得到1ms中断：
 *          Prescaler = 72-1 = 71
 *          Period = 1000-1 = 999
 *          中断周期 = (71+1)*(999+1)/72,000,000 = 0.001秒 = 1ms
 */
void TIM2_Init(u16 Prescaler, u16 Period)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 1. 使能TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 2. 配置TIM2
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = Period;           // 自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = Prescaler;     // 预分频值
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    // 3. 清除更新中断标志
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    
    // 4. 使能TIM2更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    // 5. 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 6. 启动TIM2
    TIM_Cmd(TIM2, ENABLE);
    
    // 7. 重置中断计数器
    tim2_interrupt_count = 0;
}

/**
 * @brief  获取TIM2中断次数（用于测试）
 */
uint32_t TIM2_GetInterruptCount(void)
{
    return tim2_interrupt_count;
}

/**
 * @brief  TIM2中断服务程序（1ms定时中断）
 * @note   负责：1. 系统滴答计数 2. 按键扫描
 */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        // 1. 更新系统滴答计数器（每1ms增加1）
        delay_inc_tick();
        
        // 2. 递增中断计数器（用于测试）
        tim2_interrupt_count++;
        
        // 3. 按键扫描（每1ms扫描一次）
        Key_scan();
        
        // 4. 清除中断标志
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}

