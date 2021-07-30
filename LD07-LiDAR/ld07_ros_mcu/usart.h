#ifndef __USART_H__
#define __USART_H__

#define Lidar_MeasureInfo_Num 320 //雷达测量数据信息
extern u8 Lidar_Cmd;
extern u8 parameters_ready;
extern u8 Lidar_Data[Lidar_MeasureInfo_Num];
extern u32 coe[4];

#endif