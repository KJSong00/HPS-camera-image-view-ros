#include "ros/ros.h"//ros
#include "hps_camera/distance.h"//msg
#include "hps_camera/camera.h"//srv
#include <string.h>
#include <signal.h>

/*ctrl + z*/
void signal_handler(int signo)
{
	if(signo == SIGTSTP)
	{
		printf("exit server\n");
		exit(0);
	}
}
//Subscribe to the callback function
void chatterCallback(const hps_camera::distance& msg)
{  
	printf("distance_average = %d\n", msg.distance_average);
} 

//Receive the client service and response
bool send_cmd(hps_camera::camera::Request  &req, hps_camera::camera::Response &res)
{
	std::stringstream scmd;
	//The request from the client
	printf("client_name: %s\n", req.client_node_name.c_str() );
	if( strcmp(req.client_node_name.c_str(), "camera_client" ) == 0 )
	{
		scmd<< "start";
		//response
		res.control_cmd = scmd.str();
		printf("send_cmd: %s\n", res.control_cmd.c_str() );
	}
	return true;
}

int main(int argc, char **argv)
{  
	//ros init
	ros::init(argc, argv, "ros_camera_server");   
	//Create a node
	ros::NodeHandle n;  
	//Create a service
	ros::ServiceServer service = n.advertiseService("client_login", send_cmd);
	//Subscribe to the topic
	ros::Subscriber sub = n.subscribe("camera", 1000, chatterCallback);
	printf("waiting client login\n");
	if(signal(SIGTSTP,signal_handler) == SIG_ERR)
	{
		printf("sigint error");
	}

	while(ros::ok())
	{
		ros::spinOnce();
	}
	printf("exit(0)\n");
	return 0;
}

