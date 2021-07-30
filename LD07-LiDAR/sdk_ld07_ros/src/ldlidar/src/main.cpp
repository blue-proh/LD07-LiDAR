#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "trnet.h"
#include <unistd.h>
#include "rtrnet.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ld07");
	ros::NodeHandle nh; 
	std::string topic_name("LD07/LDLiDAR");
	ros::Publisher lidar_pub = nh.advertise<sensor_msgs::LaserScan>(topic_name, 1); /*create a ROS topic */
	CmdInterfaceLinux cmd_port;
	std::vector<std::pair<std::string, std::string>> device_list;
	std::string port_name;
	cmd_port.GetCmdDevices(device_list);
	RTRNet *lidar = new RTRNet();

	for (auto n : device_list)
	{
		std::cout << n.first << "    " << n.second << std::endl;
		if (strstr(n.second.c_str(), "CP2102"))
		{
			port_name = n.first;
		}
	}

	if (port_name.empty())
		std::cout << "CANNOT FIND LiDAR_LD07" << std::endl;

	if(cmd_port.Open(port_name))//检查port口是否打开
	{
		std::cout << "FOUND LiDAR_LD07" << std::endl;//找到LiDAR_LD07
	}else
	{
		std::cout << "CANNOT FIND LiDAR_LD07" << std::endl;//没有找到LiDAR_LD07
		return (-1);
	}
	
	//回调函数
	cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
		lidar->UnpackData((uint8_t *)byte, len);
	});

	int error = 0;
	while (lidar->IsParametersReady() == false) //等待lidar启动，没有启动之前向lidar发送一系列设置指令
	{
		//cmd_port是连接雷达的port，PACK_CONFIG_ADDRESS地址配置
		lidar->SendCmd(cmd_port, 0, PACK_CONFIG_ADDRESS);
		sleep(1);
		//如果只连接了一个 LD07 设备，则无需发送配置地址命令，该设备的地址默认为 0x01
		//THIS_DEVICE_ADDREESS是设备地址，PACK_GET_COE获取校正参数
		lidar->SendCmd(cmd_port, THIS_DEVICE_ADDREESS, PACK_GET_COE);
		sleep(1);
		//PACK_VIDEO_SIZE获得相机分辨率
		lidar->SendCmd(cmd_port, THIS_DEVICE_ADDREESS, PACK_VIDEO_SIZE);
		error++;//如果lidar启动过久则直接跳出
		if (error > 2) /* Exit if the number of errors is more than 2*/
		{
			std::cout << "Error: GET LD07 PARAMETERS FAILED" << std::endl;  //Error: GET LD07 PARAMETERS FAILED
			return -1;
		}
	}
	std::cout << "get param successfull!!" << std::endl;  //get param successfull!!
	lidar->SendCmd(cmd_port, THIS_DEVICE_ADDREESS, PACK_GET_DISTANCE);  //PACK_GET_DISTANCE获取测量数据
	std::cout << "SEND PACK_GET_DISTANCE CMD" << std::endl;  //SEND PACK_GET_DISTANCE CMD

	while (ros::ok())
	{
		if (lidar->IsFrameReady())
		{
			lidar_pub.publish(lidar->GetLaserScan());  //发布获得的数据
			lidar->ResetFrameReady();
		}
	}

	return 0;
}
