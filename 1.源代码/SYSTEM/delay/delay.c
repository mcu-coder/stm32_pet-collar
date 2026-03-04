#include "delay.h"
#include "stm32f10x.h"

static volatile uint32_t system_tick = 0; // 系统滴答计数器（毫秒级）

/**
 * @brief  初始化延迟函数（TIM2已作为系统滴答定时器）
 * @param  SYSCLK: 系统时钟频率（单位：MHz，如72）
 */
void delay_init(u8 SYSCLK)
{
    // TIM2已经在TIM2.c中初始化并开启中断
    // 这里只需要初始化system_tick
    system_tick = 0;
    
    // 配置SysTick用于阻塞延时（不开启中断）
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
    
    // 计算延时倍数（用于阻塞延时）
    // 72MHz时，HCLK/8=9MHz，所以1us=9个计数
    // 这个计算只是为了阻塞延时的精确性，与system_tick无关
}

/**
 * @brief  获取当前系统时间（单位：ms）
 * @retval 系统启动后的毫秒数
 */
uint32_t delay_get_tick(void)
{
    return system_tick;
}

/**
 * @brief  递增系统滴答计数器（由TIM2中断调用）
 */
void delay_inc_tick(void)
{
    system_tick++;
}

/**
 * @brief  阻塞式毫秒延时（使用SysTick，不依赖中断）
 * @param  nms: 延时毫秒数
 */
void delay_ms(u16 nms)
{
    u32 temp;
    
    // 临时关闭SysTick中断（确保不会冲突）
    u32 ctrl_backup = SysTick->CTRL;
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    
    // 加载延时计数
    // 72MHz/8=9MHz，1ms=9000个计数
    SysTick->LOAD = (u32)nms * 9000;
    SysTick->VAL = 0x00;           // 清空计数器
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // 使能，不使用中断
    
    // 等待计数完成
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16)));
    
    // 恢复SysTick配置
    SysTick->CTRL = ctrl_backup;
    SysTick->VAL = 0x00;  // 清空计数器
}

/**
 * @brief  阻塞式微秒延时
 * @param  nus: 延时微秒数
 */
void delay_us(u32 nus)
{
    u32 temp;
    
    u32 ctrl_backup = SysTick->CTRL;
    SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    
    // 72MHz/8=9MHz，1us=9个计数
    SysTick->LOAD = nus * 9;
    SysTick->VAL = 0x00;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16)));
    
    SysTick->CTRL = ctrl_backup;
    SysTick->VAL = 0x00;
}

