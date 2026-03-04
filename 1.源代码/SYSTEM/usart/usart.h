#ifndef __USART_H
#define __USART_H

#include "sys.h"
#include "GPS.h"  // 包含GPS.h，获取宏定义
#include <stdio.h>
#include <string.h>

// 串口核心宏定义（仅保留一份，无重复）
#define USART_REC_LEN   200  	// 最大接收字节数（≥100）
#define EN_USART1_RX    1		// 使能串口1接收

// 串口接收缓冲区声明（供中断使用）
extern char USART_RX_BUF[USART_REC_LEN];
extern u16 USART_RX_STA;         	// 接收状态标记	
extern char rxdatabufer;
extern u16 point1;

// 串口核心函数声明（删除冗余的clrStruct、_SaveData相关）
void uart_init(u32 bound);
void CLR_Buf(void);
u8 Hand(char *a);



#endif

