#include "GPS.h"
#include "usart2.h"
#include "oled.h"   // OLED显示相关头文件
#include "delay.h"  // 延时/滴答定时器头文件
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// 需在GPS.h中定义的宏（若已定义可删除）
#ifndef GPS_BUF_MAX_LEN
#define GPS_BUF_MAX_LEN    128    // GPS缓冲区最大长度
#endif

#ifndef LATITUDE_MAX_LEN
#define LATITUDE_MAX_LEN   16     // 纬度字符串最大长度
#endif

#ifndef LONGITUDE_MAX_LEN
#define LONGITUDE_MAX_LEN  16     // 经度字符串最大长度
#endif

#ifndef HEMISPHERE_MAX_LEN
#define HEMISPHERE_MAX_LEN 2      // 半球方向（N/S/E/W）最大长度
#endif

// 新增：30秒无数据超时宏定义
#define GPS_NO_DATA_TIMEOUT 30000 // 30秒无新数据显示No Signal
#define GPS_STATUS_RESET_TIME 35000 // 35秒无数据复位GPS状态（避免与30秒显示冲突）

// 全局变量（需在GPS.h中声明 extern）
GPS_DataTypedef gps_data = {0};
float gps_lat_decimal = 0.0f;
float gps_lon_decimal = 0.0f;

// 保存第一次有效GPS数据
float gps_first_lat_decimal = 0.0f;   // 第一次纬度（十进制度）
float gps_first_lon_decimal = 0.0f;   // 第一次经度（十进制度）
char gps_first_ns_hem[HEMISPHERE_MAX_LEN] = {0}; // 第一次纬度半球
char gps_first_ew_hem[HEMISPHERE_MAX_LEN] = {0}; // 第一次经度半球
uint8_t is_first_data_captured = 0;   // 首次数据捕获标记：0=未捕获 1=已捕获
uint32_t last_screen_update_time = 0; // 屏幕最后更新时间戳（用于5秒更新）
uint32_t gps_last_new_data_time = 0;  // 新增：最后一次获取有效GPS数据的时间戳

// 外部变量（来自主程序，判断当前模式和页面）
extern uint8_t mode;               // 系统模式：1=自动 2=手动 3=设置
extern uint8_t auto_page;          // 自动模式页面：1=传感器页 2=GPS页
#define AUTO_MODE 1                // 自动模式标识（与主程序一致）

// 静态变量
static uint32_t gps_last_valid_time = 0;
static uint8_t gps_signal_lost_counter = 0;

// 静态函数声明
static float NMEA2Decimal(const char* nmea_str, bool is_longitude);
static uint8_t ParseNMEA_Data(const char* buffer);
// 新增：校验经纬度和半球信息的有效性
static uint8_t GPS_CheckDataValid(float lat, float lon, const char* ns, const char* ew);

/******************************************************************
 * 函数名：GPS_Init
 * 功能：GPS模块初始化（清空缓冲区、重置所有状态标志）
 ******************************************************************/
void GPS_Init(void) {
    memset(&gps_data, 0, sizeof(GPS_DataTypedef));
    gps_data.is_data_ready = false;
    gps_data.is_parsed = false;
    gps_data.is_valid = false;
    gps_lat_decimal = 0.0f;
    gps_lon_decimal = 0.0f;
    gps_last_valid_time = 0;
    gps_signal_lost_counter = 0;
    
    // 初始化首次数据相关变量
    gps_first_lat_decimal = 0.0f;
    gps_first_lon_decimal = 0.0f;
    memset(gps_first_ns_hem, 0, HEMISPHERE_MAX_LEN);
    memset(gps_first_ew_hem, 0, HEMISPHERE_MAX_LEN);
    is_first_data_captured = 0;
    last_screen_update_time = 0;
    gps_last_new_data_time = 0; // 初始化新增时间戳
}

/******************************************************************
 * 函数名：GPS_CheckDataValid
 * 功能：校验经纬度数值和半球信息的有效性（新增）
 ******************************************************************/
static uint8_t GPS_CheckDataValid(float lat, float lon, const char* ns, const char* ew) {
    // 1. 经纬度数值范围合法
    if (lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f) {
        return 0;
    }
    // 2. 经纬度数值非零（排除无效的0值）
    if (lat == 0.0f && lon == 0.0f) {
        return 0;
    }
    // 3. 半球信息合法（N/S 或 E/W）
    if ((ns[0] != 'N' && ns[0] != 'S') || (ew[0] != 'E' && ew[0] != 'W')) {
        return 0;
    }
    return 1; // 数据有效
}

/******************************************************************
 * 函数名：ParseNMEA_Data
 * 功能：解析NMEA数据（GGA/RMC/GLL），提取经纬度和半球信息
 ******************************************************************/
static uint8_t ParseNMEA_Data(const char* buffer) {
    char temp_buf[GPS_BUF_MAX_LEN];
    strncpy(temp_buf, buffer, GPS_BUF_MAX_LEN - 1); // 留空终止符位置
    temp_buf[GPS_BUF_MAX_LEN - 1] = '\0';          // 强制终止
    
    char* token;
    char* fields[15] = {0};
    uint8_t idx = 0;
    
    token = strtok(temp_buf, ",");
    while (token != NULL && idx < 15) {
        fields[idx++] = token;
        token = strtok(NULL, ",");
    }
    
    if (idx < 7) return 0; // 字段数不足，解析失败
    
    // 清空原有经纬度和半球信息（仅解析阶段，失败后不会更新全局有效数据）
    memset(gps_data.latitude, 0, LATITUDE_MAX_LEN);
    memset(gps_data.longitude, 0, LONGITUDE_MAX_LEN);
    memset(gps_data.ns_hemisphere, 0, HEMISPHERE_MAX_LEN);
    memset(gps_data.ew_hemisphere, 0, HEMISPHERE_MAX_LEN);
    gps_data.is_valid = false;
    
    // 解析GGA语句（优先）
    if (strstr(buffer, "GGA")) {
        if (fields[6] && fields[6][0] == '1') { // 定位有效
            gps_data.is_valid = true;
            if (fields[2]) strncpy(gps_data.latitude, fields[2], LATITUDE_MAX_LEN - 1);
            if (fields[3]) strncpy(gps_data.ns_hemisphere, fields[3], HEMISPHERE_MAX_LEN - 1);
            if (fields[4]) strncpy(gps_data.longitude, fields[4], LONGITUDE_MAX_LEN - 1);
            if (fields[5]) strncpy(gps_data.ew_hemisphere, fields[5], HEMISPHERE_MAX_LEN - 1);
        }
    } 
    // 解析RMC语句（备用）
    else if (strstr(buffer, "RMC")) {
        if (fields[2] && fields[2][0] == 'A') { // 定位有效
            gps_data.is_valid = true;
            if (fields[3]) strncpy(gps_data.latitude, fields[3], LATITUDE_MAX_LEN - 1);
            if (fields[4]) strncpy(gps_data.ns_hemisphere, fields[4], HEMISPHERE_MAX_LEN - 1);
            if (fields[5]) strncpy(gps_data.longitude, fields[5], LONGITUDE_MAX_LEN - 1);
            if (fields[6]) strncpy(gps_data.ew_hemisphere, fields[6], HEMISPHERE_MAX_LEN - 1);
        }
    }
    // 解析GLL语句（备用）
    else if (strstr(buffer, "GLL")) {
        if (fields[6] && fields[6][0] == 'A') { // 定位有效
            gps_data.is_valid = true;
            if (fields[1]) strncpy(gps_data.latitude, fields[1], LATITUDE_MAX_LEN - 1);
            if (fields[2]) strncpy(gps_data.ns_hemisphere, fields[2], HEMISPHERE_MAX_LEN - 1);
            if (fields[3]) strncpy(gps_data.longitude, fields[3], LONGITUDE_MAX_LEN - 1);
            if (fields[4]) strncpy(gps_data.ew_hemisphere, fields[4], HEMISPHERE_MAX_LEN - 1);
        }
    }
    
    return gps_data.is_valid;
}

/******************************************************************
 * 函数名：GPS_ParseNMEA
 * 功能：解析NMEA协议核心语句，提取关键信息并转换为十进制度
 * 修改点：解析失败时不更新全局有效数据和时间戳，保护正确数据
 ******************************************************************/
void GPS_ParseNMEA(void) {
    if (!gps_data.is_data_ready) {
        return;
    }
    
    // 保存原始数据并清空就绪标志
    char temp_buf[GPS_BUF_MAX_LEN];
    strncpy(temp_buf, gps_data.buf, GPS_BUF_MAX_LEN - 1);
    temp_buf[GPS_BUF_MAX_LEN - 1] = '\0';
    gps_data.is_data_ready = false;
    
    // 检查是否包含有效定位语句
    if (!strstr(temp_buf, "GGA") && !strstr(temp_buf, "RMC") && !strstr(temp_buf, "GLL")) {
        gps_signal_lost_counter++;
        gps_data.is_valid = false; // 无有效语句，标记为无效
        gps_data.is_parsed = true;
        printf("GPS解析失败：无有效NMEA语句\r\n");
        return;
    }
    
    // 解析NMEA数据
    if (ParseNMEA_Data(temp_buf)) {
        // 度分格式转换为十进制度
        float current_lat = NMEA2Decimal(gps_data.latitude, false);
        float current_lon = NMEA2Decimal(gps_data.longitude, true);
        
        // 范围合法性检查（纬度±90，经度±180）
        if ((current_lat < -90.0f || current_lat > 90.0f) || 
            (current_lon < -180.0f || current_lon > 180.0f)) {
            gps_signal_lost_counter++;
            gps_data.is_valid = false;
            printf("GPS解析失败：经纬度数值范围非法\r\n");
        } else {
            // 数据有效性最终校验
            if (GPS_CheckDataValid(current_lat, current_lon, gps_data.ns_hemisphere, gps_data.ew_hemisphere)) {
                // 仅当数据完全有效时，才更新全局有效数据
                gps_lat_decimal = current_lat;
                gps_lon_decimal = current_lon;
                gps_last_valid_time = delay_get_tick();
                gps_data.is_valid = true;
                gps_signal_lost_counter = 0;
                gps_last_new_data_time = delay_get_tick(); // 记录最后一次有效数据时间
                
                // 首次捕获有效数据时保存
                if (!is_first_data_captured) {
                    gps_first_lat_decimal = current_lat;
                    gps_first_lon_decimal = current_lon;
                    strncpy(gps_first_ns_hem, gps_data.ns_hemisphere, HEMISPHERE_MAX_LEN - 1);
                    strncpy(gps_first_ew_hem, gps_data.ew_hemisphere, HEMISPHERE_MAX_LEN - 1);
                    is_first_data_captured = 1;
                    printf("首次捕获GPS有效数据：纬度%.5f%c, 经度%.5f%c\r\n", 
                           gps_first_lat_decimal, gps_first_ns_hem[0],
                           gps_first_lon_decimal, gps_first_ew_hem[0]);
                    last_screen_update_time = delay_get_tick(); // 首次捕获立即更新屏幕
                }
                // 调试打印
                printf("GPS解析成功: 纬度%.5f%c, 经度%.5f%c\r\n", 
                       current_lat, gps_data.ns_hemisphere[0],
                       current_lon, gps_data.ew_hemisphere[0]);
            } else {
                gps_signal_lost_counter++;
                gps_data.is_valid = false;
                printf("GPS解析失败：数据有效性校验不通过\r\n");
            }
        }
    } else {
        gps_signal_lost_counter++;
        gps_data.is_valid = false;
        printf("GPS解析失败：NMEA语句解析失败\r\n");
    }
    
    gps_data.is_parsed = true;
}

/******************************************************************
 * 函数名：GPS_DisplayAndSend
 * 功能：GPS显示逻辑重构：解析失败时显示存入的正确数据，不显示错误数据
 ******************************************************************/
void GPS_DisplayAndSend(void) {
    // 仅在自动模式第二页执行
    if (mode != AUTO_MODE || auto_page != 2) {
        return;
    }

    char lat_str[12] = {0};
    char lon_str[12] = {0};
    uint32_t current_time = delay_get_tick();

    // 逻辑1：未捕获到首次数据，强制显示No Signal
    if (!is_first_data_captured) {
        OLED_ShowString(45, 16, (uint8_t*)"No Signal ", 16, 1);
        OLED_ShowString(45, 32, (uint8_t*)"No Signal ", 16, 1);
    }
    // 逻辑2：已捕获首次数据（有正确数据存入）
    else {
        // 子逻辑A：30秒内无新有效数据，显示No Signal
        if (current_time - gps_last_new_data_time >= GPS_NO_DATA_TIMEOUT) {
            OLED_ShowString(45, 16, (uint8_t*)"No Signal ", 16, 1);
            OLED_ShowString(45, 32, (uint8_t*)"No Signal ", 16, 1);
            printf("GPS30秒无新数据，显示No Signal\r\n");
        }
        // 子逻辑B：30秒内有过有效数据（无论当前解析是否成功）
        else {
            // 子逻辑B1：当前解析成功且数据有效，每5秒更新一次屏幕（显示最新正确数据）
            if (gps_data.is_valid) {
                if (current_time - last_screen_update_time >= 5000) {
                    snprintf(lat_str, sizeof(lat_str), "%.5f%c", gps_lat_decimal, gps_data.ns_hemisphere[0]);
                    snprintf(lon_str, sizeof(lon_str), "%.5f%c", gps_lon_decimal, gps_data.ew_hemisphere[0]);
                    OLED_ShowString(45, 16, (uint8_t*)lon_str, 16, 1);
                    OLED_ShowString(45, 32, (uint8_t*)lat_str, 16, 1);
                    last_screen_update_time = current_time; // 更新时间戳
                    printf("GPS屏幕更新：最新经纬度\r\n");
                }
            }
            // 子逻辑B2：当前解析失败（数据无效），显示首次保存的正确数据
            else {
                snprintf(lat_str, sizeof(lat_str), "%.5f%c", gps_first_lat_decimal, gps_first_ns_hem[0]);
                snprintf(lon_str, sizeof(lon_str), "%.5f%c", gps_first_lon_decimal, gps_first_ew_hem[0]);
                OLED_ShowString(45, 16, (uint8_t*)lon_str, 16, 1);
                OLED_ShowString(45, 32, (uint8_t*)lat_str, 16, 1);
                printf("GPS解析失败，显示首次保存的正确数据\r\n");
            }
        }
    }

    // 蓝牙发送逻辑：仅当数据有效时发送（解析失败时不发送）
    static uint32_t last_send_time = 0;
    if (gps_data.is_valid && (current_time - last_send_time > 3000)) {
        GPS_SendLatLonViaUSART2();
        last_send_time = current_time;
    }
}

/******************************************************************
 * 函数名：NMEA2Decimal
 * 功能：NMEA度分格式转换为十进制度
 ******************************************************************/
static float NMEA2Decimal(const char* nmea_str, bool is_longitude) {
    if (nmea_str == NULL || strlen(nmea_str) < 5) {
        return 0.0f;
    }

    char* dot_ptr = strchr(nmea_str, '.');
    if (dot_ptr == NULL) {
        return 0.0f;
    }

    uint8_t deg_digits = is_longitude ? 3 : 2;
    if ((dot_ptr - nmea_str) < deg_digits + 1) {
        return 0.0f;
    }

    char deg_str[4] = {0};
    strncpy(deg_str, nmea_str, deg_digits);
    float deg = atof(deg_str);

    char min_str[8] = {0};
    strncpy(min_str, nmea_str + deg_digits, dot_ptr - (nmea_str + deg_digits) + 4);
    float min = atof(min_str);

    return deg + (min / 60.0f);
}

/******************************************************************
 * 函数名：GPS_SendLatLonViaUSART2
 * 功能：通过串口2（蓝牙）发送经纬度数据
 ******************************************************************/
void GPS_SendLatLonViaUSART2(void) {
    char bluetooth_buf[64] = {0};
    char lat_hem = (gps_data.ns_hemisphere[0] != '\0') ? gps_data.ns_hemisphere[0] : ' ';
    char lon_hem = (gps_data.ew_hemisphere[0] != '\0') ? gps_data.ew_hemisphere[0] : ' ';
    
    snprintf(bluetooth_buf, sizeof(bluetooth_buf), 
             "纬度:%.5f%c,经度:%.5f%c\r\n",
             gps_lat_decimal, lat_hem,
             gps_lon_decimal, lon_hem);
    USART2_SendString((const char*)bluetooth_buf);
}

/******************************************************************
 * 函数名：GPS_ClearBuffer
 * 功能：彻底清空GPS缓冲区和状态标志
 ******************************************************************/
void GPS_ClearBuffer(void) {
    memset(gps_data.buf, 0, sizeof(gps_data.buf));
    memset(gps_data.utc_time, 0, sizeof(gps_data.utc_time));
    memset(gps_data.latitude, 0, sizeof(gps_data.latitude));
    memset(gps_data.longitude, 0, sizeof(gps_data.longitude));
    memset(gps_data.ns_hemisphere, 0, sizeof(gps_data.ns_hemisphere));
    memset(gps_data.ew_hemisphere, 0, sizeof(gps_data.ew_hemisphere));
    gps_data.is_data_ready = false;
    gps_data.is_parsed = false;
    gps_data.is_valid = false;
    gps_signal_lost_counter = 0;
}

/******************************************************************
 * 函数名：GPS_Status_Check
 * 功能：GPS状态检查，长时间无信号时复位状态（调整为35秒）
 ******************************************************************/
void GPS_Status_Check(void) {
    static uint32_t last_check_time = 0;
    uint32_t current_time = delay_get_tick();
    
    if (current_time - last_check_time < 1000) return;
    last_check_time = current_time;
    
    // 超过35秒无有效数据，复位GPS（避免与30秒显示超时冲突）
    if (gps_last_valid_time > 0 && (current_time - gps_last_valid_time > GPS_STATUS_RESET_TIME)) {
        printf("GPS35秒无信号，复位状态\r\n");
        GPS_Init();
        gps_last_valid_time = 0;
        gps_signal_lost_counter = 0;
    }
}

/******************************************************************
 * 函数名：GPS_Debug
 * 功能：GPS调试信息打印（增加解析失败和有效数据来源信息）
 ******************************************************************/
void GPS_Debug(void) {
    static uint32_t last_debug_time = 0;
    uint32_t current_time = delay_get_tick();
    // 新增：定义有符号变量存储超时剩余时间，避免符号变化
    int32_t timeout_remaining = -1;
    
    if (current_time - last_debug_time > 1000) {
        last_debug_time = current_time;
        
        // 计算30秒超时剩余时间（显式类型转换为有符号整数）
        if (gps_last_new_data_time != 0) {
            timeout_remaining = (int32_t)GPS_NO_DATA_TIMEOUT - (int32_t)(current_time - gps_last_new_data_time);
        } else {
            timeout_remaining = -1;
        }
        
        printf("===== GPS调试信息 =====\r\n");
        printf("数据就绪：%d, 已解析：%d, 有效：%d\r\n",
               gps_data.is_data_ready, gps_data.is_parsed, gps_data.is_valid);
        printf("首次捕获：%d | 纬度原始：%s %s, 转换后：%.5f%c\r\n",
               is_first_data_captured, gps_data.latitude, gps_data.ns_hemisphere, 
               gps_lat_decimal, gps_data.ns_hemisphere[0]);
        printf("经度原始：%s %s, 转换后：%.5f%c\r\n",
               gps_data.longitude, gps_data.ew_hemisphere, 
               gps_lon_decimal, gps_data.ew_hemisphere[0]);
        printf("首次纬度：%.5f%c, 首次经度：%.5f%c\r\n",
               gps_first_lat_decimal, gps_first_ns_hem[0],
               gps_first_lon_decimal, gps_first_ew_hem[0]);
        // 替换原有行：使用预计算的有符号变量，消除类型混合运算
        printf("最后有效数据时间：%lu ms | 30秒超时剩余：%ld ms\r\n",
               gps_last_new_data_time, timeout_remaining);
        printf("无信号计数：%d, 最后有效时间：%lu ms\r\n",
               gps_signal_lost_counter, gps_last_valid_time);
        printf("========================\r\n");
    }
}


