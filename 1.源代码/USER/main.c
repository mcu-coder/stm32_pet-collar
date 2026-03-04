#include "stm32f10x.h"                  // 引入STM32F10x系列单片机核心库头文件
#include "led.h"                        // 引入LED驱动相关头文件
#include "beep.h"                       // 引入蜂鸣器驱动相关头文件
#include "usart.h"                      // 引入串口1（调试/打印）驱动头文件
#include "delay.h"                      // 引入延时函数相关头文件
#include "oled.h"                       // 引入OLED显示屏驱动相关头文件
#include "key.h"                        // 引入按键驱动相关头文件
#include "Modules.h"                    // 引入模块相关结构体定义头文件
#include "adcx.h"                       // 引入ADC采集（光照等）相关头文件
#include "flash.h"                      // 引入FLASH存储（阈值/时间）相关头文件
#include "usart2.h"                     // 引入串口2（蓝牙通信）驱动头文件
#include "usart3.h"                     // 引入串口3（语音模块）驱动头文件
#include "TIM2.h"                       // 引入定时器2（定时中断）驱动头文件
#include "timer.h"                      // 引入定时器相关头文件
#include "GPS.h"                        // 引入GPS模块（定位/经纬度）驱动头文件
#include "ds18b20.h"                    // 引入DS18B20温度传感器驱动头文件
#include "max30102_read.h"              // 引入MAX30102心率血氧传感器驱动头文件
#include "myiic.h"                      // 引入IIC总线通信驱动头文件
 

// 蓝牙状态枚举
#define BT_STATE_INIT          0   // 蓝牙状态：初始化
#define BT_STATE_CONNECTED     1   // 蓝牙状态：已连接
#define BT_STATE_DISCONNECTED  2   // 蓝牙状态：已断开

// 蓝牙管理器结构体（对应BT_Manager）
typedef struct {
    uint32_t last_ack_time;          // 最后一次收到心跳确认的时间戳
    uint32_t last_disconnect_time;   // 最后一次蓝牙断开的时间戳
    uint8_t state;                   // 蓝牙当前状态
    uint8_t timeout_cnt;             // 蓝牙超时计数
    uint32_t last_heartbeat_time;    // 最后一次发送心跳包的时间戳(ms)
} BT_Manager;                        // 蓝牙管理结构体定义


#define KEY_Long1	11             
#define KEY_1	1                  
#define KEY_2	2                   
#define KEY_3	3                   
#define KEY_4	4                   


#define FLASH_START_ADDR	0x0801f000	

/**************** 纯软件蓝牙检测宏定义 ****************/
#define HEARTBEAT_CMD    "HEART\r\n"  // 蓝牙心跳包发送指令
#define HEARTBEAT_ACK    "2"             // 蓝牙心跳包确认响应
#define HEARTBEAT_INTERVAL 1000          // 蓝牙心跳包发送间隔(ms)
#define BT_TIMEOUT_MS     10000          // 蓝牙断开超时阈值（10秒）


SensorModules sensorData;
SensorThresholdValue Sensorthreshold;
DriveModules driveData;

uint8_t mode = 1;	                
u8 dakai;                           
u8 Flag_dakai;                     
uint8_t is_secondary_menu = 0;      
uint8_t secondary_pos = 1;          
uint8_t secondary_type = 0;        

// 外部变量：GPS解析后的纬度十进制值
extern float gps_lat_decimal;
// 外部变量：GPS解析后的经度十进制值
extern float gps_lon_decimal;
static uint8_t count_m = 1;         // 手动模式当前选中控制项：1-灯光 2-蜂鸣器
static uint8_t count_s = 1;         // 设置模式当前选中阈值项：1-温度 2-心率 3-血氧 4-光照
uint8_t auto_page = 1;              // 自动模式当前显示页面：1-传感器页 2-GPS页

extern unsigned char p[16];

// 蓝牙管理结构体
BT_Manager bt_manager;

// 静态变量：标记是否首次进入自动模式GPS页面
static uint8_t first_enter_auto_page2 = 0;

// 系统模式枚举定义
enum 
{
	AUTO_MODE = 1,       // 自动模式
	MANUAL_MODE,         // 手动模式
	SETTINGS_MODE        // 设置模式
} MODE_PAGES;            // 模式页面枚举别名

/**
  * @brief  显示菜单1的固定内容
  * @param  无
  * @retval 显示页面标题、传感器数据项名称
  */
void OLED_autoPage1(void)		
{
    // OLED显示"温度"
	OLED_ShowChinese(0,0,0,16,1); 
	OLED_ShowChinese(16,0,1,16,1);
	OLED_ShowChar(32,0,':',16,1);   
	
    // OLED显示"心率"
	OLED_ShowChinese(0,16,2,16,1); 
	OLED_ShowChinese(16,16,3,16,1);
	OLED_ShowChar(32,16,':',16,1);  
 
	
	OLED_Refresh(); 
}

// 自动模式菜单第二页（GPS数据页）
void OLED_autoPage2(void)   
{
    OLED_ShowString(45, 0, "G P S", 16, 1); 
	
    // OLED显示"纬度"
	OLED_ShowChinese(0,16,10,16,1);	
	OLED_ShowChinese(16,16,9,16,1);	
    OLED_ShowChar(32,16,':',16,1);   
	
     
    GPS_DisplayAndSend(); // 将GPS数据显示到OLED并通过蓝牙发送
    first_enter_auto_page2 = 1;      // 标记首次进入GPS页面
    
    OLED_Refresh();       
}

/**
  * @brief  自动模式第一页传感器数据显示+蓝牙发送
  * @param  无
  * @retval 无
  * @note   1. 采集传感器数据（光照、距离、跌倒检测）
  *         2. 解析GPS数据（若有新数据），格式化经纬度
  *         3. 合并传感器数据+GPS数据，通过蓝牙发送
  *         4. 显示时间、光照、距离、跌倒检测结果到OLED
  */
void SensorDataDisplay1(void)		
{
    char all_data[256] = {0};  // 传感器数据拼接缓冲区
    char gps_str[64] = {0};    // GPS数据格式化缓冲区

    // 如果GPS有新数据，执行解析操作
    if (gps_data.is_data_ready) {
        GPS_ParseNMEA();
    }
   
    // 合并传感器数据和GPS数据为统一字符串
    sprintf(all_data, "温度: %.1f C\r\n心率：%d \r\n 血氧：%d\r\n光照：%d\r\n%s",
    sensorData.temp, sensorData.hrAvg, sensorData.spo2Avg, sensorData.lux, gps_str);
            
    USART2_SendString((const char*)all_data);  // 通过串口2（蓝牙）发送合并后的数据
	
     
}

/**
  * @brief  自动模式第二页GPS数据显示+蓝牙发送
  * @param  无
  * @retval 无
  * @note   1. 格式化光照、距离数据，通过蓝牙发送
  *         2. 解析GPS数据，显示经纬度到OLED+蓝牙发送
  */
void SensorDataDisplay2(void) {
    static uint32_t last_gps_time = 0; // 上次处理GPS数据的时间戳(ms)
    uint32_t current_time = delay_get_tick(); // 获取当前系统时间戳(ms)
    
    // 核心逻辑：首次进入GPS页面时，立即处理GPS数据（跳过500ms限制）
    if (first_enter_auto_page2) {
        if (gps_data.is_data_ready) {
            GPS_ParseNMEA();    // 解析GPS原始数据
        }
        GPS_DisplayAndSend();   // 显示并发送GPS数据
        last_gps_time = current_time; // 更新上次GPS处理时间戳
        first_enter_auto_page2 = 0; // 重置首次进入标志
        return;
    }
    
    // 非首次进入：按500ms频率限制处理GPS数据（避免频繁解析）
    if (current_time - last_gps_time > 500) {
        if (gps_data.is_data_ready) {
            GPS_ParseNMEA();    // 解析GPS原始数据
        }
        GPS_DisplayAndSend();   // 显示并发送GPS数据
        last_gps_time = current_time; // 更新上次GPS处理时间戳
    }
}

/**
  * @brief  显示手动模式设置界面1
  * @param  无
  * @retval 无
  */
void OLED_manualPage1(void)
{
    // OLED显示"灯光"
	OLED_ShowChinese(16,0,12,16,1);	
	OLED_ShowChinese(32,0,13,16,1);	
	OLED_ShowChinese(48,0,14,16,1);	
	OLED_ShowChar(64,0,':',16,1);   
 
}

/**
  * @brief  显示手动模式设置参数界面1
  * @param  无
  * @retval 无
  */
void ManualSettingsDisplay1(void)
{
    // 根据LED控制标志显示
	if(driveData.LED_Flag ==1)
	{
		OLED_ShowChinese(96,0,18,16,1); 
	}
	 
	 
}

/**
  * @brief  显示系统阈值设置界面1
  * @param  无
  * @retval 无
  */
void OLED_settingsPage1(void)
{
    // OLED显示"温度阈值"
	OLED_ShowChinese(16,0,0,16,1);	
	OLED_ShowChinese(32,0,1,16,1);	
	OLED_ShowChinese(48,0,20,16,1);	
	OLED_ShowChinese(64,0,21,16,1);
    OLED_ShowChar(80,0,':',16,1);   

    // OLED显示"心率阈值"
	OLED_ShowChinese(16,16,2,16,1);	
	OLED_ShowChinese(32,16,3,16,1);	
	OLED_ShowChinese(48,16,20,16,1);	
	OLED_ShowChinese(64,16,21,16,1);	
	OLED_ShowChar(80,16,':',16,1);   
	
    // OLED显示"血氧阈值"
	OLED_ShowChinese(16,32,4,16,1);	
	OLED_ShowChinese(32,32,5,16,1);	
	OLED_ShowChinese(48,32,20,16,1);	
	OLED_ShowChinese(64,32,21,16,1);	
	OLED_ShowChar(80,32,':',16,1);  
	 
}

// 显示系统阈值设置界面2
void OLED_settingsPage2(void)
{
}

// 显示系统阈值设置界面3
void OLED_settingsPage3(void)
{
}

// 显示系统阈值设置界面1的实际数值
void SettingsThresholdDisplay1(void)
{
    // OLED显示温度阈值数值
	OLED_ShowNum(90, 0, Sensorthreshold.tempValue , 3,16,1);
    // OLED显示心率阈值数值
	OLED_ShowNum(90, 16, Sensorthreshold.hrAvgValue , 3,16,1);
   
}

// 显示系统阈值设置界面2的实际数值
void SettingsThresholdDisplay2(void)
{
}

// 显示系统阈值设置界面3的实际数值
void SettingsThresholdDisplay3(void)
{
}

/**
  * @brief  自动模式页面切换处理（按键2）
  * @param  无
  * @retval 当前自动模式页面（1=第一页，2=第二页）
  * @note   按下KEY2切换自动模式页面，切换时清屏并绘制对应页面固定内容
  */
uint8_t SetAuto(void)  
{
    // 检测到KEY2按下时执行页面切换
	  if(KeyNum == KEY_2)  
    {
        KeyNum = 0;         
        // 页面切换逻辑：1→2，2→1
        auto_page = (auto_page == 1) ? 2 : 1;  
        OLED_Clear();        // OLED清屏（避免页面内容重叠）
        delay_ms(5);         // 清屏后短暂延时，避免OLED绘制残留
        
        // 如果切换到GPS页面，标记首次进入
		    if(auto_page == 2) {
            first_enter_auto_page2 = 1;
        }
			
       
    }
    return auto_page;  // 返回当前自动模式页面
}

/**
  * @brief  手动模式控制项切换处理（按键2）
  * @param  无
  * @retval 当前手动模式控制项（1=灯光，2=蜂鸣器）
  * @note   按下KEY2切换控制项，超过2项则循环到1
  */
uint8_t SetManual(void)  
{
    // 检测到KEY2按下时执行控制项切换
	if(KeyNum == KEY_2)  
	{
		KeyNum = 0;  // 清除按键标志
		count_m++;   // 控制项计数+1
	 
	}
	return count_m;  // 返回当前手动模式控制项
}

/**
  * @brief  设置模式控制项切换处理（按键2）
  * @param  无
  * @retval 当前设置模式控制项（1=时间，2=距离阈值，3=光照阈值）
  * @note   按下KEY2切换设置项，超过3项则循环到1
  */
uint8_t SetSelection(void)
{
    
    return count_s;  // 返回当前设置模式控制项
}

/**
  * @brief  显示手动模式界面的选择符号
  * @param  num 为显示的位置
  * @retval 无
  */
void OLED_manualOption(uint8_t num)
{
   
	switch(num)
	{
		case 1:	// 选中灯光控制项
			OLED_ShowChar(0, 0,'>',16,1);
			OLED_ShowChar(0,16,' ',16,1);
			OLED_ShowChar(0,32,' ',16,1);
			OLED_ShowChar(0,48,' ',16,1);
			break;
		 
	}
}

/**
  * @brief  显示阈值界面的选择符号
  * @param  num 为显示的位置
  * @retval 无
  */
void OLED_settingsOption(uint8_t num)
{
    static uint8_t prev_num = 1; // 记录上一次光标位置（避免重复绘制）

    // 清除上一次光标（仅操作光标位置，不影响数据显示）
    switch(prev_num)
    {
        case 1: OLED_ShowChar(0, 0, ' ', 16, 1); break; // 温度阈值行
        case 2: OLED_ShowChar(0, 16, ' ', 16, 1); break; // 心率阈值行
       
        default: break;
    }
    // 根据当前选中项显示光标">"
	switch(num)
	{
		case 1:	// 选中温度阈值项
			OLED_ShowChar(0, 0,'>',16,1);
			OLED_ShowChar(0,16,' ',16,1);
			OLED_ShowChar(0,32,' ',16,1);
			OLED_ShowChar(0,48,' ',16,1);
			break;
		case 2:	// 选中心率阈值项
			OLED_ShowChar(0, 0,' ',16,1);
			OLED_ShowChar(0,16,'>',16,1);
			OLED_ShowChar(0,32,' ',16,1);
			OLED_ShowChar(0,48,' ',16,1);
			break;
	 
		default: break;
	}
	prev_num = num; // 更新上一次光标位置
    OLED_Refresh(); // 仅刷新光标，数据区域无变化
}

/**
  * @brief  自动模式控制函数
  * @param  无
  * @retval 无
  */
void AutoControl(void)
{	
    // 光照强度低于阈值时，开启LED灯
    if(sensorData.lux<Sensorthreshold.luxValue)
		driveData.LED_Flag =1;
    else
		driveData.LED_Flag =0; 

     
    // 将阈值参数写入FLASH（掉电保存）
	FLASH_W(FLASH_START_ADDR,Sensorthreshold.tempValue,Sensorthreshold.hrAvgValue,Sensorthreshold.spo2AvgValue, Sensorthreshold.luxValue);
}

/**
  * @brief  手动模式控制函数
  * @param  无
  * @retval 无
  */
void ManualControl(uint8_t num)
{
    // 根据选中的控制项执行按键操作
	switch(num)
	{
		case 1:  // 灯光控制项
            if(KeyNum == KEY_3) // KEY3开启LED
            {
                driveData.LED_Flag = 1;  
                KeyNum = 0;  
                printf("[按键] KEY3按下，LED_Flag置1\n");  
            }
            if(KeyNum == KEY_4) // KEY4关闭LED
            {
                driveData.LED_Flag = 0; 
                KeyNum = 0;  
                printf("[按键] KEY4按下，LED_Flag置0\n"); 
            }
            break;

		case 2:  // 蜂鸣器控制项
            if(KeyNum == KEY_3)  // KEY3开启蜂鸣器
            {
                KeyNum = 0;
                if (driveData.BEEP_Flag != 1)  // 避免重复动作
                {
                    driveData.BEEP_Flag = 1;
                }
            }
            if(KeyNum == KEY_4)  // KEY4关闭蜂鸣器
            {
                KeyNum = 0;
                if (driveData.BEEP_Flag != 0)  
                {
                    driveData.BEEP_Flag = 0;
                }
            }
            break;
		default: break;
	}
}

/**
  * @brief  硬件设备控制函数
  * @param  无
  * @retval 无
  */
void Control_Manager(void)
{
    // 根据LED控制标志控制LED开关
    if(driveData.LED_Flag )
    {	
        LED_On(); 
    }
    else 
    {
        LED_Off();
    }
		
    
}

/**
  * @brief  阈值设置函数
  * @param  无
  * @retval 无
  */
void ThresholdSettings(uint8_t num)
{
    // 根据选中的阈值项执行加减操作
	switch (num)
	{
		// 温度阈值设置
		case 1:
			if (KeyNum == KEY_3) // KEY3增加温度阈值
			{
				KeyNum = 0;
				Sensorthreshold.tempValue += 1;
				// 温度阈值上限限制（40℃），超过则重置为10℃
				if (Sensorthreshold.tempValue > 40)
				{
					Sensorthreshold.tempValue = 10;
				}
			}
		 
			break;
			
		// 心率阈值设置
		case 2:
			if (KeyNum == KEY_3) // KEY3增加心率阈值
			{
				KeyNum = 0;
				Sensorthreshold.hrAvgValue += 1;
				// 心率阈值上限限制（120次/分），超过则重置为70
				if (Sensorthreshold.hrAvgValue > 120)
				{
					Sensorthreshold.hrAvgValue = 70;
				}
			}
		 
			break;
		  
		
        default: break;
	}   
}

// 从FLASH读取保存的阈值参数
void FLASH_ReadThreshold()
{
   
    Sensorthreshold.tempValue = FLASH_R(FLASH_START_ADDR );       
    Sensorthreshold.hrAvgValue = FLASH_R(FLASH_START_ADDR + 2);  
    Sensorthreshold.spo2AvgValue = FLASH_R(FLASH_START_ADDR + 4);  
	  Sensorthreshold.luxValue = FLASH_R(FLASH_START_ADDR + 6);  
}

/**
  * @brief  主函数（程序入口）
  * @param  无
  * @retval int：返回值（实际未使用）
  * @note   1. 初始化所有硬件模块
  *         2. 读取FLASH保存的参数（阈值、时间）
  *         3. 主循环：处理按键、模式切换、传感器数据、设备控制、显示刷新
  */
int main(void)
{ 
    SystemInit();               // 配置系统时钟为72MHz	
    delay_init(72);             // 延时函数初始化（基于72MHz系统时钟）
	  TIM2_Init(72-1, 1000-1);    // 定时器2初始化（定时1ms中断）
    ADCX_Init();                // ADC初始化（用于光照强度采集）
    LED_Init();                 // LED初始化（GPIO配置）
    
    OLED_Init();                // OLED显示屏初始化
	  GPS_Init();                 // GPS模块初始化（缓冲区清空、状态重置）
    Init_MAX30102();            // MAX30102心率血氧传感器初始化
     

    // 初始化蓝牙管理器
    uint32_t current_time = delay_get_tick();
    bt_manager.last_ack_time = current_time;          // 初始化最后心跳确认时间
    bt_manager.last_disconnect_time = current_time;   // 初始化最后断开时间
    bt_manager.state = BT_STATE_INIT;                 // 初始化蓝牙状态为未初始化
    bt_manager.timeout_cnt = 0;                       // 初始化超时计数
    bt_manager.last_heartbeat_time = 0;               // 初始化最后心跳发送时间

    delay_ms(100);            // 短暂延时，等待硬件稳定
    FLASH_ReadThreshold();    // 从FLASH读取保存的阈值参数
    OLED_Clear();             // OLED清屏（初始化后清除残影）
   
    // 状态管理静态变量
    static uint8_t last_mode = 0;             // 记录上一次系统模式
    static uint32_t last_sensor_time = 0;     // 上次传感器采集的时间戳
    static uint32_t last_display_time = 0;    // 上次OLED刷新的时间戳
    
    // 参数有效性检查：若阈值超出合理范围，重置为默认值
    if (Sensorthreshold.tempValue > 40 || Sensorthreshold.hrAvgValue > 120 || 
       Sensorthreshold.spo2AvgValue > 100 || Sensorthreshold.luxValue > 500)
    {
        
        FLASH_W(FLASH_START_ADDR, 30, 95, 98, 100);
        FLASH_ReadThreshold(); // 重新读取阈值 
    }
    
    // 系统启动提示：串口1打印、语音模块播报
    printf("系统启动，蓝牙初始状态: 初始化\n");
    USART3_SendString("AF:30");    // 语音模块：设置音量30
    delay_ms(200);                 // 短暂延时，避免指令冲突
    USART3_SendString("A7:00001"); // 语音模块：播报系统启动
    delay_ms(200);                 // 短暂延时，等待语音播报完成

    // 主循环（程序核心逻辑）
    while (1)
    {	
        uint32_t current_time = delay_get_tick(); // 获取当前系统时间戳(ms)
        
        
        // ==================== 传感器数据采集（100ms一次） ====================
        if(current_time - last_sensor_time > 100) {
            SensorScan();             // 采集所有传感器数据
			    last_sensor_time = current_time; // 更新上次采集时间戳
		    }
        
        // ==================== 立即处理按键事件 ====================
        uint8_t current_key_num = KeyNum; // 保存当前按键值（避免被多次处理）
        
        // 模式切换按键处理（KEY1=模式切换，KEY_Long1=自动→设置）
        if(current_key_num != 0)
        {
            switch(mode)
            {
                case AUTO_MODE: // 当前是自动模式
                    if(current_key_num == KEY_1) // KEY1短按：自动→手动
                    {
                        mode = MANUAL_MODE;
                        count_m = 1;           // 手动模式默认选中灯光
                        driveData.LED_Flag = 0; // 切换时关闭LED
                        driveData.BEEP_Flag = 0; // 切换时关闭蜂鸣器
                        KeyNum = 0;            // 清除按键标志
                    }
                    else if(current_key_num == KEY_Long1) // KEY1长按：自动→设置
                    {
                        mode = SETTINGS_MODE;
                        count_s = 1;           // 设置模式默认选中温度阈值
                        KeyNum = 0;            // 清除按键标志
                    }
                    break;
                    
                case MANUAL_MODE: // 当前是手动模式
                    if(current_key_num == KEY_1) // KEY1短按：手动→自动
                    {
                        mode = AUTO_MODE;
                        auto_page = 1;         // 切回自动模式传感器页
                        KeyNum = 0;            // 清除按键标志
                    }
                    break;
                    
                case SETTINGS_MODE: // 当前是设置模式（按键在内部处理）
                    break;
            }
        }
        
        // 模式切换检测：模式变化时清屏并绘制新页面
        if(last_mode != mode)
        {
            OLED_Clear(); // OLED清屏（避免模式间内容重叠）
            last_mode = mode; // 更新上一次模式记录
            
            // 绘制新模式的固定页面内容
            switch(mode)
            {
                case AUTO_MODE:
                    OLED_autoPage1(); // 自动模式默认显示传感器页
                    break;
                case MANUAL_MODE:
                    OLED_manualPage1(); // 手动模式显示控制界面
                    break;
                case SETTINGS_MODE:
                    OLED_settingsPage1(); // 设置模式显示阈值界面
                    break;
            }
            OLED_Refresh(); // 立即刷新OLED显示
        }
        
        // 按当前系统模式执行对应逻辑
        switch(mode)
        {
            case AUTO_MODE: // 自动模式
            {
                // 获取当前自动模式页面（处理KEY2切换）
                uint8_t curr_auto_page = SetAuto();
                if(curr_auto_page == 1)
                {
                    SensorDataDisplay1();	// 显示传感器数据+蓝牙发送
                }
                else
                {
                    SensorDataDisplay2();	// 显示GPS数据+蓝牙发送
                }
                
                AutoControl();     // 执行自动控制逻辑（LED/蜂鸣器）
                Control_Manager(); // 执行硬件设备控制
                break;
            }    
            
            case MANUAL_MODE: // 手动模式
            {
                // 手动模式状态管理静态变量
                static uint8_t manual_page_initialized = 0; // 页面初始化标志
                static uint8_t last_manual_count = 0;       // 上一次控制项
                static uint8_t last_LED_Flag = 0;           // 上一次LED状态
                static uint8_t last_BEEP_Flag = 0;          // 上一次蜂鸣器状态
                static uint8_t force_refresh = 0;           // 强制刷新标志
                
                // 模式切换时初始化状态
                if(last_mode != mode)
                {
                    manual_page_initialized = 0;
                    last_manual_count = 0;
                    last_LED_Flag = driveData.LED_Flag;
                    last_BEEP_Flag = driveData.BEEP_Flag;
                    force_refresh = 1;  // 强制刷新显示
                    count_m = 1;        // 默认选中灯光控制
                    driveData.LED_Flag = 0; // 初始关闭LED
                    driveData.BEEP_Flag = 0; // 初始关闭蜂鸣器
                }
                
                // 获取当前手动模式控制项（处理KEY2切换）
                uint8_t current_manual_count = SetManual();
                
                // 检测设备状态是否变化（变化则需要刷新显示）
                uint8_t need_refresh = 0;
                if(driveData.LED_Flag != last_LED_Flag || driveData.BEEP_Flag != last_BEEP_Flag)
                {
                    need_refresh = 1;
                    last_LED_Flag = driveData.LED_Flag;
                    last_BEEP_Flag = driveData.BEEP_Flag;
                }
                
                // 页面未初始化/控制项变化/状态变化/强制刷新时，重新绘制页面
                if(!manual_page_initialized || current_manual_count != last_manual_count || need_refresh || force_refresh)
                {
                    OLED_manualPage1();          // 绘制手动模式固定内容
                    OLED_manualOption(current_manual_count); // 绘制选中光标
                    ManualSettingsDisplay1();    // 绘制设备状态（开/关）
                    manual_page_initialized = 1; // 标记页面已初始化
                    last_manual_count = current_manual_count; // 更新控制项记录
                    force_refresh = 0;           // 清除强制刷新标志
                    OLED_Refresh(); // 刷新OLED显示
                }
                
              
				
               
                Control_Manager(); // 执行硬件设备控制
                break;
            }
            
            case SETTINGS_MODE: // 设置模式
            {
                // 设置模式页面初始化标志
                static uint8_t is_threshold_page_inited = 0;
                // 获取当前设置模式控制项（处理KEY2切换）
                uint8_t curr_count_s = SetSelection();
                
                // 立即处理设置模式内的按键
                if(current_key_num != 0)
                {
                    if (is_secondary_menu == 1)
                    {
                        // 二级菜单按键处理（暂未实现）
                        if (current_key_num == KEY_2 || current_key_num == KEY_3 || current_key_num == KEY_4)
                        {
                            OLED_Refresh(); // 刷新显示
                            KeyNum = 0;     // 清除按键标志
                        }
                        else if (current_key_num == KEY_1)
                        {
                            // KEY1返回一级菜单
                            is_secondary_menu = 0;
                            secondary_pos = 1;
                            OLED_Clear(); // 清屏
                            OLED_settingsPage1(); // 绘制一级菜单
                            SettingsThresholdDisplay1(); // 绘制阈值数值
                            OLED_settingsOption(curr_count_s); // 绘制光标
                            OLED_Refresh(); // 刷新显示
                            KeyNum = 0;     // 清除按键标志
                        }
                    }
                    else
                    {
                        // 一级菜单按键处理
                        if (current_key_num == KEY_3 || current_key_num == KEY_4)
                        {
                            ThresholdSettings(curr_count_s); // 调整阈值
                            SettingsThresholdDisplay1();     // 刷新阈值数值
                            OLED_Refresh(); // 刷新显示
                            KeyNum = 0;     // 清除按键标志
                        }
                      
                    }
                }
                
                
                break;
            }
        }
        
        // OLED刷新频率控制：每50ms刷新一次（避免频繁刷新占用资源）
        if(current_time - last_display_time > 50) {
            OLED_Refresh();          // 刷新OLED显示
            last_display_time = current_time; // 更新上次刷新时间戳
        }
    }
}


