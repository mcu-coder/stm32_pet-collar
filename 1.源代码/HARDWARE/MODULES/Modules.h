#ifndef	__MODULES_H_
#define __MODULES_H_

#include "stm32f10x.h"                  // Device header
 //#include "dht11.h"
#include "adcx.h"

void MPU6050_InitModule(void);  // 初始化MPU6050
void MPU6050_UpdateData(void);  // 更新MPU6050数据及状态

typedef struct
{
    int32_t hrAvg;      // 心率
    int32_t spo2Avg;    // 血氧 
    uint8_t humi;
    float temp;
    uint16_t lux;	
    uint16_t soilHumi;
    uint16_t Smoge;	
    uint16_t AQI;
    uint16_t CO;
    uint16_t hPa;
    uint8_t distance;
	
    int16_t ax, ay, az;    // 加速度
    int16_t gx, gy, gz;    // 角速度
    int16_t sum;           // 角速度总和差值
    uint8_t mpu6050_id;    // MPU6050 ID
    uint8_t mpu_test;      // 碰撞检测标志（0=无，1=有）
	 uint8_t temp_valid;   // 温度有效标志：0=无效（初始85℃），1=有效（真实温度）
	 int16_t accel_avg;          // 单次三轴算术平均（AX+AY+AZ)/3
    int16_t accel_avg_filtered; // 滑动平均优化后的值（更稳定）
} SensorModules;
typedef struct
{
	float tempValue;
	uint8_t humiValue;
	//uint8_t tempValue;
	uint16_t luxValue;	




	uint16_t SmogeValue;

	uint16_t hrAvgValue;
	uint16_t spo2AvgValue;


	
}SensorThresholdValue;

typedef struct
{
  //uint8_t	NOW_LED_Flag;
	uint8_t LED_Flag;
	uint8_t BEEP_Flag;
	uint8_t NOW_Curtain_Flag;
	uint8_t Curtain_Flag;	
	uint8_t NOW_Window_Flag;
	uint8_t Window_Flag;	
	uint8_t Fan_Flag;
	uint8_t Humidifier_Flag;
	uint8_t Bump_Flag;
  uint8_t MPU_Flag;
	
}DriveModules;
 	// 初始化驱动数据（全局变量）


extern SensorModules sensorData;			//声明传感器模块的结构体变量
extern SensorThresholdValue Sensorthreshold;	//声明传感器阈值结构体变量
extern DriveModules driveData;				//声明驱动器状态的结构体变量
void SensorScan(void);
#endif


