#include "ros/ros.h"//ros
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "hps_camera/distance.h"//msg
#include "hps_camera/camera.h"//srv
#include "../include/api.h"//api interface
#include <sstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>


HPS3D_HandleTypeDef handle;

ros::Publisher camera_pub;//Global variable, because the observer callback function needs to be used

//The observer callback function
void *User_Func(HPS3D_HandleTypeDef *handle, AsyncIObserver_t *event)
{
	hps_camera::distance msg;
	if(event->AsyncEvent == ISubject_Event_DataRecvd)
	{
		switch(event->RetPacketType)
		{
			case SIMPLE_ROI_PACKET:
				printf("1\n");
				printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.simple_roi_data[0].distance_average,event->RetPacketType);
				break;
			case FULL_ROI_PACKET:
				printf("2\n");
				//Assign the average distance to the MSG message
				msg.distance_average = event->MeasureData.full_roi_data[0].distance_average;
				printf("distance = %d\n",msg.distance_average);
				//Publish MSG messages
				camera_pub.publish(msg);
				break;
			case FULL_DEPTH_PACKET:
				printf("3\n");
				printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.full_depth_data->distance_average,event->RetPacketType);
				break;
			case SIMPLE_DEPTH_PACKET:
				printf("4\n");
				printf("distance = %d  event->RetPacketType = %d\n",event->MeasureData.simple_depth_data->distance_average,event->RetPacketType);
				break;
			case NULL_PACKET:
				printf("5\n");
				//The return packet type is empty
				break;
			default:
				printf("system error!\n");
				break;
		}
	}
	return 0;
}

//check ctrl+c signal
void signal_handler(int signo)
{
    if(HPS3D_RemoveDevice(&handle) != RET_OK)
    {
		printf("HPS3D_RemoveDevice faild\n");
	}
    else
    {	
        printf("HPS3D_RemoveDevice succeed\n");
    }
    exit(0);
}


//printf log callback function
void my_printf(uint8_t *str)
{
	std::cout<< str;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "ros_camera_client");//ros init
	ros::NodeHandle n;//Create a node

	uint32_t a = 0;
	uint8_t fileName[10][20];
	uint32_t dev_cnt = 0;
	RET_StatusTypeDef ret = RET_OK;
	AsyncIObserver_t My_Observer;

	std::stringstream sclient_name;
	hps_camera::distance msg;

	//Install the signal
	if(signal(SIGINT,signal_handler) == SIG_ERR)
	{
		printf("sigint error");
	}
	if(signal(SIGTSTP,signal_handler) == SIG_ERR)
	{
		printf("sigint error");
	}
	//Create a service
	ros::ServiceClient client = n.serviceClient<hps_camera::camera>("client_login");
	hps_camera::camera srv;
	sclient_name<<"camera_client";
	printf("send name = %s\n",sclient_name.str().c_str());
	srv.request.client_node_name = sclient_name.str();
	//Create a topic
	camera_pub = n.advertise<hps_camera::distance>("camera", 1000);	

	//set debug enable and install printf log callback function
	HPS3D_SetDebugEnable(true);
	HPS3D_SetDebugFunc(&my_printf);

	//Observer callback function and initialization
	My_Observer.AsyncEvent = ISubject_Event_DataRecvd;
	My_Observer.NotifyEnable = true;
	My_Observer.ObserverID = 2;
	My_Observer.RetPacketType = NULL_PACKET;

	//Lists the optional devices
	dev_cnt = HPS3D_GetDeviceList((uint8_t *)"/dev/",(uint8_t *)"ttyACM",fileName);
	printf("Current connectable device (please select)：\n");
	for(uint32_t i=0;i<dev_cnt;i++)
	{
		printf("%d: %s\n",i,fileName[i]);
	}
	printf("Please enter the corresponding serial number：\n");
	scanf("%d",&a);
	handle.DeviceName = fileName[a];

	do
	{
		//Device Connection
		ret = HPS3D_Connect(&handle);
		if(ret != RET_OK)
		{
			printf("Device open failed！ret = %d\n",ret);
			break;
		}
		//Device init
		ret = HPS3D_ConfigInit(&handle);
		if(RET_OK != ret)
		{
			printf("Initialization failed:%d\n", ret);
			break;
		}
		printf("Initialization succeed\n");

		//The actual invocation of the service
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
		printf("login succeed!\n");

		//Add observer one
		HPS3D_AddObserver(&User_Func, &handle, &My_Observer);		


		//Set running mode
		//handle.RunMode = RUN_CONTINUOUS;
		//HPS3D_SetRunMode(&handle);
	}while(0);

	if(ret != RET_OK)
	{
		//Remove device and disconnect
		HPS3D_RemoveDevice(&handle);
		printf("Initialization failed, Remove device\n");
		return 0;
	}

	HPS3D_GetPacketType(&handle);
	printf("1handle packet type = %d\n", handle.OutputPacketType);
	handle.OutputPacketType = PACKET_FULL;
	HPS3D_SetPacketType(&handle);
	HPS3D_GetPacketType(&handle);
	printf("2handle packet type = %d\n", handle.OutputPacketType);

	// init publisher and image message
	ros::Publisher imagePub = n.advertise<sensor_msgs::Image>("/hps3d_image", 100);
	sensor_msgs::Image image;
	image.header.stamp = ros::Time::now();
	image.height = 60;
	image.width = 160;
	image.encoding = "mono8";
	image.is_bigendian = true;
	image.step = image.width;
	ros::Rate loopRate(5);

	while(ros::ok())
	{
		HPS3D_SingleMeasurement(&handle);
		switch(handle.RetPacketType)
		{
			case SIMPLE_ROI_PACKET:
				msg.distance_average = handle.MeasureData.simple_roi_data[0].distance_average;	
				printf("1\n");
				printf("distance = %d\n",msg.distance_average);			
				//Publish MSG messages
				camera_pub.publish(msg);
				break;
			case FULL_ROI_PACKET:
				//Assign the average distance to the MSG message
				msg.distance_average = handle.MeasureData.full_roi_data[0].distance_average;
				printf("2\n");
				printf("distance = %d\n",msg.distance_average);
				//Publish MSG messages
				camera_pub.publish(msg);
				break;
			// test only here
			case FULL_DEPTH_PACKET:		
				msg.distance_average = handle.MeasureData.full_depth_data->distance_average;
				printf("distance = %d\n",msg.distance_average);				
				camera_pub.publish(msg);
				// make image and publish
				for(int i = 0; i < (image.height * image.width); i++)
				{	
					image.data.push_back(handle.MeasureData.full_depth_data->distance[i] / 60);
				}
				imagePub.publish(image);
				image.data.clear();
				break;
			case SIMPLE_DEPTH_PACKET:
				msg.distance_average = handle.MeasureData.simple_depth_data->distance_average;	
				printf("4\n");
				printf("distance = %d\n",msg.distance_average);			
				//Publish MSG messages
				camera_pub.publish(msg);
				break;
			case NULL_PACKET:
				//The return packet type is empty
				break;
			default:
				printf("system error!\n");
				break;
		}
		loopRate.sleep();
		//Waiting to receive
		//ros::spinOnce();
	}

	return 0;
}

