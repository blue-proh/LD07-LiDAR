#include "uart.h"

float Lidar_Distance[Lidar_MeasureInfo_Num / 2];
float Lidar_Theta[Lidar_MeasureInfo_Num / 2];

void TransformSignlePoint(u16 dist, int n, int fine_num, u8 confidence)
{
	double k0 = (double)(coe[0]) / 10000.0;
	double k1 = (double)(coe[1]) / 10000.0;
	double b0 = (double)(coe[2]) / 10000.0;
	double b1 = (double)(coe[3]) / 10000.0;
	const double pi = 3.14159265;
	double pixel_u = n, tmp_theta, tmp_dist, tmp_x, tmp_y, Theta, Distance;
	
	if (pixel_u > 80)
	{
		pixel_u = pixel_u - 80;
		pixel_u = 80 - pixel_u;
		if (b0 > 1) //217之前的版本计算的b值在20-30之间，217之后的版本计算的b值小于1;
		{
			tmp_theta = k0 * pixel_u - b0;
		}
		else
		{
			tmp_theta = atan(k0 * pixel_u - b0) * 180 / pi;
		}
		tmp_dist = (dist - 1.22) / cos((22.5 - (tmp_theta)) * pi / 180);
		tmp_theta = tmp_theta * pi / 180;
		tmp_x = cos(22.5 * pi / 180) * tmp_dist * cos(tmp_theta) + sin(22.5 * pi / 180) * (tmp_dist * sin(tmp_theta));
		tmp_y = -sin(22.5 * pi / 180) * tmp_dist * cos(tmp_theta) + cos(22.5 * pi / 180) * (tmp_dist * sin(tmp_theta));
		tmp_x = tmp_x + 1.22;
		tmp_y = tmp_y - 5.315;
		//Lidar_Distance[fine_num] = sqrt(tmp_x * tmp_x + tmp_y * tmp_y);
		//Lidar_Theta[fine_num] = atan(tmp_y / tmp_x) * 180 / pi;
		Lidar_Distance[n] = sqrt(tmp_x * tmp_x + tmp_y * tmp_y);
		Lidar_Theta[n] = atan(tmp_y / tmp_x) * 180 / pi;
	}
	else
	{
		pixel_u = 80 - pixel_u;
		if (b1 > 1)
		{
			tmp_theta = k1 * pixel_u - b1;
		}
		else
		{
			tmp_theta = atan(k1 * pixel_u - b1) * 180 / pi;
		}
		tmp_dist = (dist - 1.22) / cos((22.5 + (tmp_theta)) * pi / 180);
		tmp_theta = tmp_theta * pi / 180;
		tmp_x = cos(-22.5 * pi / 180) * tmp_dist * cos(tmp_theta) + sin(-22.5 * pi / 180) * (tmp_dist * sin(tmp_theta));
		tmp_y = -sin(-22.5 * pi / 180) * tmp_dist * cos(tmp_theta) + cos(-22.5 * pi / 180) * (tmp_dist * sin(tmp_theta));
		tmp_x = tmp_x + 1.22;
		tmp_y = tmp_y + 5.315;
		Lidar_Distance[n] = sqrt(tmp_x * tmp_x + tmp_y * tmp_y);
		Lidar_Theta[n] = atan(tmp_y / tmp_x) * 180 / pi;
	}
	if (Theta < 0)
	{
		Theta += 360;
	}

}

u16 Lidar_Successful_Num = 0;
void Lidar_Transform(u8 *Lidar_Tr_Data)
{
	int n = 0; //一共160个理论上的点
	int fine_num = 0; //实际测量到距离的点的个数
	for (int i = 0; i < Lidar_MeasureInfo_Num; i += 2, n++)
	{
		u16 value = ((Lidar_Tr_Data[i + 1] << 8) | Lidar_Tr_Data[i]);
		u8 confidence = (value >> 9) << 1;  //计算置信度
		value &= 0x1ff;  //计算测量距离
		u16 center_dis = value;
		//if (center_dis > 0)
		//{
			//直接读取所有点
			TransformSignlePoint(center_dis, n, fine_num, confidence);
			//fine_num++;
		//}
			if (i == Lidar_MeasureInfo_Num - 2)
				Lidar_Successful_Num++;
	}
	
}

int main(void)
{
    /*****************初始化激光雷达*******************/
	u8 err_count = 0;
	u8 data_len = 11;
	while (parameters_ready == 0)
	{
		//如果只连接了一个 LD07 设备，则无需发送配置地址命令，该设备的地址默认为 0x01
		u8 PACK_CONFIG_ADDRESS_Buff[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0x00, 0x16, 0x00, 0x00, 0x00, 0x00, 0x16 };
		Lidar_Cmd = 0x16;
		USART3_Send(PACK_CONFIG_ADDRESS_Buff, data_len);
		delay_ms(1);

		//获取校正参数命令                               设备地址 命令码
		u8 PACK_GET_COE_Buff[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0x01, 0x12, 0x00, 0x00, 0x00, 0x00, 0x13 };
		Lidar_Cmd = 0x12;
		USART3_Send(PACK_GET_COE_Buff, data_len);
		delay_ms(1);

		//获得相机分辨率                                   设备地址 命令码                       校验码：设备地址与命令码之和  
		u8 PACK_VIDEO_SIZE_Buff[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0x01, 0x15, 0x00, 0x00, 0x00, 0x00, 0x16 };
		Lidar_Cmd = 0x15;
		USART3_Send(PACK_VIDEO_SIZE_Buff, data_len);
		delay_ms(1);

		err_count++;//如果lidar启动过久则直接跳出
		//if (err_count > 2) /* Exit if the number of errors is more than 2*/
			//return -1;
	}  
	//获取测量数据                                          设备地址 命令码
	u8 PACK_GET_DISTANCE_Buff[] = { 0xAA, 0xAA, 0xAA, 0xAA, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x03 };
	Lidar_Cmd = 0x02;
	USART3_Send(PACK_GET_DISTANCE_Buff, data_len);
	delay_ms(1);

    while(1)
    {
        Lidar_Transform(Lidar_Data);
    }
}