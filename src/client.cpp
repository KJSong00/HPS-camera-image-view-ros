#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hps_camera/distance.h"//msg
#include "hps_camera/camera.h"//srv
#include "../include/api.h"
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>


HPS3D_HandleTypeDef handle;

ros::Publisher camera_pub;

void *User_Func(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event)
{
	hps_camera::distance msg;
	if(event->AsyncEvent == ISubject_Event_DataRecvd)
	{
		switch(event->RetPacketType)
		{
			case SIMPLE_ROI_PACKET:
				printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.simple_roi_data[0].distance_average,event->RetPacketType);
				break;
			case FULL_ROI_PACKET:
				//printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.full_roi_data[0].distance_average,event->RetPacketType);
				msg.distance_average = event->MeasureData.full_roi_data[0].distance_average;
				//printf("1distance = %d\n",event->MeasureData.full_roi_data[0].distance_average);
				printf("distance = %d\n",msg.distance_average);
				camera_pub.publish(msg);
				break;
			case FULL_DEPTH_PACKET:
				printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.full_depth_data->distance_average,event->RetPacketType);
				break;
			case SIMPLE_DEPTH_PACKET:
				printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.simple_depth_data->distance_average,event->RetPacketType);
				break;
			case NULL_PACKET:
				/*返回数据包类型为空*/
				break;
			default:
				printf("system error!\n");
				break;
		}
		event->RetPacketType  = NULL_PACKET;
	}
	return 0;
}


void signal_handler(int signo)
{
	while(HPS3D_RemoveDevice(&handle) != RET_OK)
	{

	}
    exit(0);
}

void my_printf(uint8_t *str)
{
	std::cout<< str;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "ros_camera_client");
	ros::NodeHandle n;
 	//ros::Rate loop_rate(1);

	uint32_t a = 0;
	uint8_t fileName[10][20];
	uint32_t dev_cnt = 0;
	RET_StatusTypeDef ret = RET_OK;
	AsyncIObserver_t My_Observer;

	std::stringstream sclient_name;
	ros::ServiceClient client = n.serviceClient<hps_camera::camera>("client_login");
	hps_camera::camera srv;
	sclient_name<<"camera_client";
	printf("send name = %s\n",sclient_name.str().c_str());
	srv.request.client_node_name = sclient_name.str();
	camera_pub = n.advertise<hps_camera::distance>("camera", 1000);
	
	
	HPS3D_SetDebugEnable(true);
	HPS3D_SetDebugFunc(&my_printf);

	/*观察者回调函数与初始化*/
	My_Observer.AsyncEvent = ISubject_Event_DataRecvd;
	My_Observer.NotifyEnable = true;
	My_Observer.ObserverID = 2;
	My_Observer.RetPacketType = NULL_PACKET;

	/*列出可选设备列表*/
	dev_cnt = HPS3D_GetDeviceList((uint8_t *)"/dev/",(uint8_t *)"ttyACM",fileName);
	printf("当前可连接的设备(请选择)：\n");
	for(uint32_t i=0;i<dev_cnt;i++)
	{
		printf("%d: %s\n",i,fileName[i]);
	}
	printf("请输入对应的序号：\n");
	scanf("%d",&a);
	handle.DeviceName = fileName[a];
	
	if(signal(SIGINT,signal_handler) == SIG_ERR)
	{
		printf("sigint error");
	}
	
	do
	{
		ret = HPS3D_Connect(&handle);
		if(ret != RET_OK)
		{
			printf("设备打开失败！ret = %d\n",ret);
			break;
		}
		/*设备初始化*/
		handle.RunMode = RUN_IDLE;
		handle.SyncMode = ASYNC;
		ret = HPS3D_ConfigInit(&handle);
		if(RET_OK != ret)
		{
			printf("初始化失败:%d\n", ret);
			break;
		}
		printf("初始化成功\n");


		if (client.call(srv))
		{
			while(ros::ok())
			{
				printf("rev cmd = %s\n",srv.response.control_cmd.c_str());
				if( strcmp(srv.response.control_cmd.c_str(), "start" ) == 0 )
				{
					break;
				}
			}
		}
		else
		{
			break;
		}



		HPS3D_AddObserver(&User_Func, &handle, &My_Observer); /*添加观察者1*/

		/*点云格式配置*/
		/*光学参数使能*/
		//HPS3D_SetOpticalEnable(&handle, true);
		/*点云数据使能*/
		//HPS3D_SetPointCloudEn(true);
	
		ROIConfTypeDef roi_conf;
		roi_conf.enable = true;
		roi_conf.left_top_x = 5;
		roi_conf.left_top_y = 5;
		roi_conf.right_bottom_x = 10;
		roi_conf.right_bottom_y = 10;
		roi_conf.roi_id = 0;
		HPS3D_SetROIRegion(&handle, roi_conf);
		HPS3D_SetROIEnable(&handle, 0, true);

		roi_conf.enable = true;
		roi_conf.left_top_x = 20;
		roi_conf.left_top_y = 10;
		roi_conf.right_bottom_x = 50;
		roi_conf.right_bottom_y = 50;
		roi_conf.roi_id = 2;
		HPS3D_SetROIRegion(&handle, roi_conf);
		HPS3D_SetROIEnable(&handle, 2, true);		


		//
		handle.RunMode = RUN_CONTINUOUS;
		HPS3D_SetRunMode(&handle);
	}while(0);

	if(ret != RET_OK)
	{
		HPS3D_RemoveDevice(&handle);
		return 0;
	}

	//HPS3D_SingleMeasurement(&handle);

	/*printf("data->RetPacketType = %d\n",packet_type);
	printf("height = %d\n",data.point_cloud_data[0].height);
	printf("width = %d\n",data.point_cloud_data[0].width);
	printf("points = %d\n",data.point_cloud_data[0].points);
	for(int i = 0; i < 60; i++)
	{
		for(int j = 0; j < 160; j++)
		{
			printf("(%d, %d, %d),",(int16_t)data.point_cloud_data[0].point_data[j + i * 160].x, (int16_t)data.point_cloud_data[0].point_data[j + i * 160].y, (uint16_t)data.point_cloud_data[0].point_data[j + i * 160].z);
		}
		printf("\n");
	}*/
	/*for(int k = 0; k < 2; k++)
	{
		printf("data->RetPacketType = %d\n",packet_type);
		printf("height = %d\n",data.point_cloud_data[k].height);
		printf("width = %d\n",data.point_cloud_data[k].width);
		printf("points = %d\n",data.point_cloud_data[k].points);

		for(int j = 0; j < data.point_cloud_data[k].points; j++)
		{
			printf("(%f, %f, %d),",data.point_cloud_data[k].point_data[j].x, data.point_cloud_data[k].point_data[j].y, (uint16_t)data.point_cloud_data[k].point_data[j].z);
			if(j % data.point_cloud_data[k].height == data.point_cloud_data[k].height-1)
			{
				printf("\n");
			}
		}
	}*/

	
	while(1)
	{
		ros::spinOnce();
		//loop_rate.sleep();
	}

	//ctrl + c退出
	//HPS3D_RemoveDevice(&handle);
	
	return 0;
}

