#include "uart.h"
#define PACK_GET_DISTANCE 0x02   /*Frame ID of distance data获取测量数据*/
#define PACK_STOP 0x0F		   /*Frame ID of stop distance data transmission停止测量数据传输*/
#define PACK_GET_COE 0x12	       /*Frame ID of Get parameters获取校正参数*/
#define PACK_VIDEO_SIZE 0x15	   /*Frame ID of Get camera resolution获得相机分辨率*/
#define PACK_CONFIG_ADDRESS 0x16 /*Frame ID of Configure address地址配置包括设备数量*/
#define Lidar_Buff_Num 400
#define Data_Start 14 //雷达数据帧RX3_buff[14]是测量信息数据的开始
#define Data_End 333 //雷达数据帧RX3_buff[333]是测量信息数据的结束

u32 usart3_data_u32 = 0;
u8 parameters_ready = 0;
u8 Lidar_Cmd = 0;
u8 Get3_date = 0;
u16 RX3_CND = 0;
u8 RX3_buff[Lidar_Buff_Num];
u8 AA_num = 0;
/*********Lidar数据************/
u32 coe[4];
u16 Lidar_Point_Num = 0;
u16 coe_u = 0;
u16 coe_v = 0;
u8 Lidar_Data[Lidar_MeasureInfo_Num];

void UART3_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART3_BASE,1);//获取中断标志 原始中断状态 屏蔽中断标志		
  UARTIntClear(UART3_BASE,flag);//清除中断标志			
  while(UARTCharsAvail(UART3_BASE))//判断FIFO是否还有数据		
  {
	  Get3_date = UARTCharGet(UART3_BASE);

	  if (RX3_CND == Lidar_Buff_Num - 1)RX3_CND = 0;//数组即将溢出

	  RX3_buff[RX3_CND++] = Get3_date;

	  if (Get3_date == 0xAA) AA_num++;
	  else AA_num = 0;

	  if (AA_num == 4)
	  {
		  AA_num = 0;
		  RX3_CND = 4;
		  RX3_buff[0] = 0xAA;
		  RX3_buff[1] = 0xAA;
		  RX3_buff[2] = 0xAA;
		  RX3_buff[3] = 0xAA;
	  }

	  switch (Lidar_Cmd)
	  {
	  case PACK_GET_DISTANCE:

		  if (RX3_CND == 335)
		  {
			  if (RX3_buff[0] == 0xAA && RX3_buff[1] == 0xAA && RX3_buff[2] == 0xAA && RX3_buff[3] == 0xAA)
			  {
				  for (int i = 0; i < Lidar_MeasureInfo_Num; i++)
				  {
					  Lidar_Data[i] = RX3_buff[i + Data_Start];  // 把雷达数据提取
				  }
			  }
		  }
		  break;

		  //获取校正参数
	  case PACK_GET_COE:

		  if (RX3_CND == 29)
		  {
			  if (RX3_buff[0] == 0xAA && RX3_buff[1] == 0xAA && RX3_buff[2] == 0xAA && RX3_buff[3] == 0xAA)
			  {

				  usart3_data_u32 = ((RX3_buff[13] << 24) | RX3_buff[12] << 16 | RX3_buff[11] << 8 | RX3_buff[10]);
				  coe[0] = usart3_data_u32;

				  usart3_data_u32 = ((RX3_buff[17] << 24) | RX3_buff[16] << 16 | RX3_buff[15] << 8 | RX3_buff[14]);
				  coe[1] = usart3_data_u32;

				  usart3_data_u32 = ((RX3_buff[21] << 24) | RX3_buff[20] << 16 | RX3_buff[19] << 8 | RX3_buff[18]);
				  coe[2] = usart3_data_u32;

				  usart3_data_u32 = ((RX3_buff[25] << 24) | RX3_buff[24] << 16 | RX3_buff[23] << 8 | RX3_buff[22]);
				  coe[3] = usart3_data_u32;

				  Lidar_Point_Num = ((RX3_buff[27] << 8) | RX3_buff[26]);
			  }
		  }
		  break;
	  
		  //获得相机分辨率
	  case PACK_VIDEO_SIZE:
		  if (RX3_CND == 14)
		  {
			  if (RX3_buff[0] == 0xAA && RX3_buff[1] == 0xAA && RX3_buff[2] == 0xAA && RX3_buff[3] == 0xAA)
			  {
				  coe_u = ((RX3_buff[11] << 8) | RX3_buff[10]);
				  coe_v = ((RX3_buff[13] << 8) | RX3_buff[12]);
				  parameters_ready = 1;
			  }
		  }
		  break;

	  default:
		  break;
	  }

  }
}