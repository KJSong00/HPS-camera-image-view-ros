/**********************************************************************
* COPYRIGHT NOTICE - HYPERSEN TECHNOLOGY
*
* Copyright (c) 2018, Hypersen Technology, Inc.
*
* All rights reserved.
*
*======================================================================
* \file api.h
* \brief TODO
* \author Kevin
* \email Kevin_Wang@hypersen.com
* \version 1.0.0
* \date 2018年11月13日 上午11:43:31
* \license private & classified
*---------------------------------------------------------------------
* Remark: This project is still under construction.
*======================================================================
* Change History:
*---------------------------------------------------------------------
* <Date>			| <Version>	| <Author>			| <Description>
*---------------------------------------------------------------------
* 2018年11月13日			| V1.0.0	| Kevin				| Create file
*======================================================================
* Detailed Notes:
*---------------------------------------------------------------------
* <Version>		| <Description>
*---------------------------------------------------------------------
* V1.0.0		| TODO
*---------------------------------------------------------------------

**********************************************************************/

#ifndef HPS3D_API_H_
#define HPS3D_API_H_

#ifdef __cplusplus
extern "C"  /*C++*/
{
#endif

#include <stdbool.h>
#include <string.h>


#ifdef _WIN32 /*windows平台*/
	#include <windows.h>
#endif

#define DLL_API _declspec(dllexport)

/*类型的宏*/
typedef signed char 	int8_t;
typedef unsigned char 	uint8_t;
typedef unsigned short 	uint16_t;
typedef short 			int16_t;
typedef unsigned int 	uint32_t;
typedef int 			int32_t;
typedef float 			float32_t;
typedef double 			float64_t;


#define 	DEV_NUM 			 (10)							/*Device number*/
#define     DEV_NAME_SIZE		 (20)							/*Device name length*/
#define 	ROI_NUM 			 (8)							/*ROI number*/
#define 	OBSTACLE_NUM 		 (20)							/*Support obstacle number*/
#define 	OBSERVER_NUM  		 (10)							/*Support observer number*/

/*	resolution	*/
#define		RES_WIDTH			 (160)							/*device pixels width*/
#define		RES_HEIGHT			 (60)							/*device pixels height*/
#define		MAX_PIX_NUM 		 (RES_WIDTH * RES_HEIGHT)		/*Total number of pixels*/

/*Special measurement data values*/
#define	 	LOW_AMPLITUDE   	（65300） 						/*The signal amplitude is low*/
#define	  	SATURATION 			（65400）    					/*Saturation potential*/
#define	 	ADC_OVERFLOW  		（65500）  						/*The ADC overflow*/
#define	 	INVALID_DATA 		（65530）   					/*Invalid data*/

/*function return status*/
typedef enum
{
	RET_OK 		= 0x01,		
	RET_ERROR 	= 0x02,		
	RET_BUSY 	= 0x03,		
	RET_CONNECT_FAILED,     
	RET_CREAT_PTHREAD_ERR,  
	RET_WRITE_ERR,          
	RET_READ_ERR,           
	RET_PACKET_HEAD_ERR,    
	RET_PACKET_ERR,    		
	RET_BUFF_EMPTY,			
	RET_VER_MISMATCH,  
}RET_StatusTypeDef;

/*device version*/
typedef struct
{
	uint8_t year;			
	uint8_t month;		
	uint8_t day;			
	uint8_t major;			
	uint8_t minor;		
	uint8_t rev;	
}Version_t;

/*Run mode*/
typedef enum
{
	MinOfRunModeType = 0,
	RUN_IDLE = 0,			/*Run to stop*/
	RUN_SINGLE_SHOT,		/*Single measurement mode*/
	RUN_CONTINUOUS,			/*Continuous measurement mode*/
	NumberOfRunModeType
}RunModeTypeDef;

/*packet type*/
typedef enum
{
	PACKET_FULL = 0,		/*Complete packet (including depth data)*/
	PACKET_SIMPLE			/*Simple packet (without depth data)*/
}OutPacketTypeDef;


/*ROI alarm type*/
typedef enum
{
	ROI_ALARM_DISABLE = 0,  /*Turn off the ROI area alert and output only information and data about the ROI area*/
	ROI_ALARM_GPIO       	/*ROI area alert type is GPIO OUT level output*/
}ROIAlarmTypeDef;

/*Hysteresis conf*/
typedef struct
{
	uint8_t threshold_id;  		    
	uint32_t threshold_value; 	
	uint32_t hysteresis; 	
	bool enable;			
	bool positive;					/*true:Forward comparison,  if the input value is greater than the threshold,returns True
									  false:Reverse comparison,  if the input value is less than the threshold, returns False*/
}HysteresisSingleConfTypeDef;

/*ROI reference type*/
typedef enum
{
	ROI_REF_DIST_AVR = 1,		
	ROI_REF_DIST_MIN,			
	ROI_REF_DIST_MAX,			
	ROI_REF_SAT_COUNT,				
	ROI_REF_AMPLITUDE,				
	ROI_REF_VAILD_AMPLITUDE,		
	ROI_REF_THRESHOLD_PIX_NUM		
}ROIReferenceTypeDef;

/*ROI param config*/
 typedef struct
{
	bool enable;										
	uint8_t roi_id;									
	uint16_t left_top_x;								
	uint16_t left_top_y;								
	uint16_t right_bottom_x;							
	uint16_t right_bottom_y;							
	HysteresisSingleConfTypeDef hysteresis_conf[3];		
	ROIReferenceTypeDef ref_type[3];				
	ROIAlarmTypeDef alarm_type[3];					
	uint16_t pixel_number_threshold[3];	
}ROIConfTypeDef;

/*HDR mode*/
typedef enum
{
	HDR_DISABLE = 0,				
	AUTO_HDR,					
	SUPER_HDR,					
	SIMPLE_HDR		
}HDRModeTypeDef;

/*HDR config*/
typedef struct
{
	HDRModeTypeDef hdr_mode;				
	float32_t qualtity_overexposed;				
	float32_t qualtity_overexposed_serious;		
	float32_t qualtity_weak;					
	float32_t qualtity_weak_serious;			
	uint32_t simple_hdr_max_integration;		
	uint32_t simple_hdr_min_integration;		
	uint8_t super_hdr_frame_number;				
	uint32_t super_hdr_max_integration;			
	uint32_t hdr_disable_integration_time;		
}HDRConf;

/*Smooth Filter*/
typedef enum
{
	SMOOTH_FILTER_DISABLE = 0,		
	SMOOTH_FILTER_AVERAGE = 1,	
	SMOOTH_FILTER_GAUSS				
}SmoothFilterTypeDef;


typedef struct
{
	SmoothFilterTypeDef type;		
	uint32_t arg1;				
}SmoothFilterConfTypeDef;

/*GPIO parameter*/
/*GPIO_OUT*/
typedef enum
{
	GPOUT_FUNC_DISABLE = 0,				
	GPOUT_FUNC_ROI_THRESHOLD0_ALARM,	
	GPOUT_FUNC_ROI_THRESHOLD1_ALARM,	
	GPOUT_FUNC_ROI_THRESHOLD2_ALARM		
}GPOutFunctionTypeDef;

/*GPIO_IN*/
typedef enum
{
	GPIN_FUNC_DISABLE = 0,			
	GPIN_FUNC_CAPTURE  
}GPInFunctionTypeDef;

/*GPIO polarity*/
typedef enum
{
	GPIO_POLARITY_LOW = 0,		
	GPIO_POLARITY_HIGH			
}GPIOPolarityTypeDef;

/*GPIO pin*/
typedef enum
{
	GPIN_1 = 1,					
	GPOUT_1 = 10	
}GPIOTypeDef;

/*GPOUT parameter*/
typedef struct
{
	GPIOTypeDef gpio;				
	GPIOPolarityTypeDef polarity;
	GPOutFunctionTypeDef function;
}GPIOOutConfTypeDef;

/*GPIN parameter*/
typedef struct
{
	GPIOTypeDef gpio;				
	GPIOPolarityTypeDef polarity;
	GPInFunctionTypeDef function;	
}GPIOInConfTypeDef;


/*distcance filter type*/
typedef enum
{
	DISTANCE_FILTER_DISABLE = 0,	
	DISTANCE_FILTER_SIMPLE_KALMAN
}DistanceFilterTypeDef;

typedef struct
{
	DistanceFilterTypeDef filter_type;		
	float32_t kalman_K; 					
	uint32_t kalman_threshold;			
	uint32_t num_check;						
}DistanceFilterConfTypeDef;

/*assemble angle parameter*/
typedef struct
{
	bool enable;					
	uint8_t angle_vertical;     	
}MountingAngleParamTypeDef;

/*Parse the packet type*/
typedef enum
{
	NULL_PACKET = 0x00,				
	SIMPLE_ROI_PACKET = 0x01,		/*Simple ROI packets (without depth map data)*/
	FULL_ROI_PACKET,				/*Complete ROI package (including depth chart data)*/
	FULL_DEPTH_PACKET,				/*Complete depth data package (including depth map data)*/
	SIMPLE_DEPTH_PACKET,			/*Simple depth packet (without depth map data)*/
	OBSTACLE_PACKET,				/*Obstacle packet */
    SYSTEM_ERROR					/*Syster error*/
}RetPacketTypedef;

/*simple ROI packet struct*/
typedef struct
{
	uint8_t group_id;						
	uint8_t id;								
	uint16_t amplitude;						
	uint16_t valid_amplitude;				
	uint16_t distance_average;				
	uint16_t distance_max;					
	uint16_t distance_min;					
	uint16_t dist_max_x;					
	uint16_t dist_max_y;					
	uint16_t dist_min_x;					
	uint16_t dist_min_y;					
	uint16_t saturation_count;				
	uint8_t threshold_state;				/*bit0:zone0, bit1:zone1, bit2:zone2*/
	uint16_t out_of_threshold_pix_num[3];	
	uint16_t frame_cnt;						
}SimpleFullRoiDataTypeDef;

/*complete ROI parameter struct*/
typedef struct
{
	uint8_t roi_num;						
	uint8_t group_id;						
	uint8_t id;								
	uint16_t left_top_x;					
	uint16_t left_top_y;				
	uint16_t right_bottom_x;				
	uint16_t right_bottom_y;			
	uint32_t pixel_number;				
	uint16_t amplitude;						
	uint16_t valid_amplitude;				
	uint16_t distance_average;				
	uint16_t distance_max;					
	uint16_t distance_min;					
	uint16_t saturation_count;				
	uint16_t threshold_state;				/*bit0:zone0, bit1:zone1, bit2:zone2*/
	uint16_t dist_max_x;					
	uint16_t dist_max_y;					
	uint16_t dist_min_x;			
	uint16_t dist_min_y;				
	uint32_t frame_cnt;						
	uint16_t distance[MAX_PIX_NUM];			/*depth data*/
}FullRoiDataTypeDef;

/*depth data*/
typedef struct
{
	uint16_t distance_average;				
	uint16_t amplitude_average;				
	uint16_t amplitude_average_whole;		
	uint16_t amplitude_low_count;			
	uint16_t saturation_count;				
	uint16_t distance_max;				
	uint16_t distance_min;					
	int16_t temperature;					
	uint16_t frame_cnt;						
	uint16_t interference_num;				
	uint16_t distance[MAX_PIX_NUM];			/*depth data*/
}DepthDataTypeDef;


/*point cloud data*/
typedef struct
{
	float32_t x;					/*x,y,z coordinates in space*/
	float32_t y;
	float32_t z;
}PerPointCloudDataTypeDef;

/*Ordered point cloud data*/
typedef struct
{
	PerPointCloudDataTypeDef point_data[MAX_PIX_NUM];	/*point cloud data */
	uint16_t width;										/*width,the number of points a row */
	uint16_t height;									/*height,line number */
	uint32_t points;									/*total points */
}PointCloudDataTypeDef;

/*Obstacle parameter*/
typedef struct
{
	bool enable;        				
	uint16_t frame_head; 	 	
	uint8_t number;  		 			
	uint16_t vaild_min_dist; 	
	uint16_t vaild_max_dist;  	
	uint16_t invaild_value;    
	uint16_t frame_size;  /*out*/  
}ObstacleConfigTypedef;

/*Obstacle data struct*/
typedef struct
{
	uint8_t Id;   									
	uint32_t FrameCount;  						
	uint16_t PixelNumber;			 			
	uint16_t DistanceAverage; 					
	PerPointCloudDataTypeDef LeftPoint; 				/*Obstacle area left endpoint coordinate value*/
	PerPointCloudDataTypeDef RightPoint; 				/*Obstacle area right endpoint coordinate value*/
	PerPointCloudDataTypeDef MinPoint;					/*The minimum point coordinate value of obstacle area*/
	PerPointCloudDataTypeDef PixelBuffer[MAX_PIX_NUM];   /*Save all pixel information buffer of obstacles*/
}ObstacleDataTypedef;


/*A struct encapsulates a struct for the return of data*/
typedef struct
{
	SimpleFullRoiDataTypeDef *simple_roi_data;	
	FullRoiDataTypeDef *full_roi_data;			
	DepthDataTypeDef *simple_depth_data;		
	DepthDataTypeDef *full_depth_data;			
	PointCloudDataTypeDef *point_cloud_data;
	ObstacleDataTypedef *Obstacle_data;			
	uint8_t *Obstacle_frame_data_buff; 			
}MeasureDataTypeDef;

/*通讯方式*/
typedef enum
{
	SYNC = 0x01,  				
	ASYNC = 0x02 						
}HPS3D_SynchronousTypedef;

/*handle*/
typedef struct
{
	uint8_t *DeviceName; 				
	uint32_t DeviceFd;  				
	uint8_t DeviceAddr; 				
	HPS3D_SynchronousTypedef SyncMode;  
	RunModeTypeDef RunMode;   			
	MeasureDataTypeDef MeasureData;     /*Synchronous measurement data, the measurement results will not be 
										saved here when asynchronous mode (can be operated on by observers)*/
	RetPacketTypedef RetPacketType;     /*Measuring the return packet type synchronously, the result of measuring the return 
										packet type asynchronously will not be saved here (it can be manipulated by observers)*/
	OutPacketTypeDef OutputPacketType; 	
	bool ConnectStatus;  	 			
	uint8_t RoiNumber;					/*Save the amount of ROI currently supported by the device*/
	uint8_t ThresholdNumber;			/*Save the number of thresholds for current device ROI support*/
}HPS3D_HandleTypeDef;

/*Optical Parameter*/
typedef struct
{
	bool enable;						
	uint8_t viewing_angle_horiz;    	
	uint8_t viewing_angle_vertical; 	
	uint8_t illum_angle_horiz;      
	uint8_t illum_angle_vertical;		
}OpticalParamConfTypeDef;

/*Interference Detect parameter*/
typedef struct
{
	bool enable;						
	uint32_t integ_time;			
	uint16_t amplitude_threshold;		
	uint8_t capture_num;				
	uint8_t number_check;	
}InterferenceDetectConfTypeDef;

/*transport type*/
typedef enum
{
	TRANSPORT_USB = 0,					/*USB*/
	TRANSPORT_CAN,						/*CAN*/
	TRANSPORT_RS232,					/*RS232*/
	TRANSPORT_RS485						/*RS485*/
}TransportTypeDef;

/*Observer subscribes to events*/
typedef enum
{
	ISubject_Event_DataRecvd = 0x01,	/*Data receive event*/
	ISubject_Event_DevConnect = 0x02,	/*device connect event*/
	ISubject_Event_DevDisconnect = 0x03 /*device disconnect event*/
}AsyncISubjectEvent;

/*The observer subscribes to the event struct parameters*/
typedef struct
{
	uint8_t ObserverID;  				
	bool NotifyEnable;   			
	AsyncISubjectEvent AsyncEvent; 		
	MeasureDataTypeDef MeasureData; 	
	RetPacketTypedef RetPacketType; 	
}AsyncIObserver_t;



/**************************************API Function*************************************/


/***********************************1.Command function interface***********************************/

/**
 * @brief	set measurement mode
 * @param[in]   handle->DeviceAddr   device address
 * @param[in]   handle->DeviceFd     device fd
 * @param[in]   handle->RunMode 	 run mode 
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetRunMode(HPS3D_HandleTypeDef *handle);

/**
 * @brief	get device address
 * @param[in]   handle->DeviceFd     device fd
 * @param[out]  handle->DeviceAddr   device address
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetDevAddr(HPS3D_HandleTypeDef *handle);

/**
 * @brief	setdevice address
 * @param[in]	handle->DeviceAddr    old device address
 * @param[in]   handle->DeviceFd      device fd
 * @param[in]	new_addr     		  new device address
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetDevAddr(HPS3D_HandleTypeDef *handle, uint8_t new_addr);

/**
 * @brief	get device info.
 * @param[in]	handle->DeviceAddr    device address
 * @param[in]   handle->DeviceFd     device fd
 * @param[out]  version_t 		 	
 * @param[out]	version_t.year		 
 * @param[out]  version_t.month	
 * @param[out]  version_t.day		 
 * @param[out]  version_t.major		 
 * @param[out]	version_t.minor		 
 * @param[out]	version_t.rev	
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetDeviceVersion(HPS3D_HandleTypeDef *handle, Version_t *version_t);

/**
 * @brief	Set the type of measurement data return package (simple package or complete package)
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @param[in]	handle->PacketType	   Select input for packet type
 * 				-PACKET_FULL	       Select the packet type and input the complete packet (including the depth data)
 *				-PACKET_SIMPLE		   Simple packets (without depth data)
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetPacketType(HPS3D_HandleTypeDef *handle);

/**
 * @brief	get output packet type
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @param[out]	handle->PacketType     Type of return packet
 * 				-PACKET_FULL	       Select the packet type and input the complete packet (including the depth data)
 *				-PACKET_SIMPLE		   Simple packets (without depth data)
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetPacketType(HPS3D_HandleTypeDef *handle);

/**
 * @brief	save to user profile
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_ProfileSaveToCurrent(HPS3D_HandleTypeDef *handle);

/**
 * @brief	clear user profile
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_ProfileClearCurrent(HPS3D_HandleTypeDef *handle);

/**
 * @brief	Restore Factory settings 
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_ProfileRestoreFactory(HPS3D_HandleTypeDef *handle);

/**
 * @brief	get transport type
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @param[out]	transport_type	 	   
 * 				-TRANSPORT_USB		   USB
 * 				-TRANSPORT_CAN		   CAN
 * 				-TRANSPORT_RS232       RS232
 * 				-TRANSPORT_RS485	   RS485
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetTransportType(HPS3D_HandleTypeDef *handle, TransportTypeDef *transport_type);

/**
 * @brief	select ROI group id
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @param[in]	group_id     		   
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SelectROIGroup(HPS3D_HandleTypeDef *handle, uint8_t group_id);

/**
 * @brief	get ROI Group ID
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @param[out]  group_id   			 
 * @note
 * @retval  RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetROIGroupID(HPS3D_HandleTypeDef *handle, uint8_t *group_id);

/**
 * @brief	set ROI alram type 
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @param[in]	group_id    	   	   
 * @param[in]	threshold_id 	   	   
 * @param[in]	roi_alarm_type     	 
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetROIAlarmType(HPS3D_HandleTypeDef *handle, uint8_t roi_id, uint8_t threshold_id, ROIAlarmTypeDef roi_alarm_type);

/**
 * @brief	set ROI reference
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @param[in]	roi_id       		   
 * @param[in]	threshold_id 		   
 * @param[in]	type	     		  
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetROIReferenceType(HPS3D_HandleTypeDef *handle, uint8_t roi_id, uint8_t threshold_id, ROIReferenceTypeDef ref_type);

/**
 * @brief	set ROI parameter
 * @param[in]	handle->DeviceAddr       device address
 * @param[in]   handle->DeviceFd         device fd
 * @param[in]	roi_conf.roi_id          
 * @param[in]	roi_conf.left_top_x		 
 * @param[in]	roi_conf.left_top_y		 
 * @param[in]	roi_conf.right_bottom_x  
 * @param[in]   roi_conf.right_bottom_y  
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetROIRegion(HPS3D_HandleTypeDef *handle, ROIConfTypeDef roi_conf);

/**
 * @brief	set ROI enable 
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @param[in]	roi_id          	   
 * @param[in]	en     				   
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetROIEnable(HPS3D_HandleTypeDef *handle, uint32_t roi_id, bool en);

/**
 * @brief	set ROI threshold 
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @param[in]	roi_id	              
 * @param[in]	threshold_id 		  
 * @param[in]	en     		 		
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetROIThresholdEnable(HPS3D_HandleTypeDef *handle, uint32_t roi_id, uint32_t threshold_id, bool en);

/**
 * @brief	set ROI parameter
 * @param[in]	handle->DeviceAddr     				device address
 * @param[in]   handle->DeviceFd            		device fd
 * @param[in]	roi_id
 * @param[in]	threshold_id 						
 * @param[in]   pix_num_threshold   				
 * @param[in]	hysteresis_conf.threshold_value		
 * @param[in]	hysteresis_conf.hysteresis  		
 * @param[in]	hysteresis_conf.positive			
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetROIThresholdConf(HPS3D_HandleTypeDef *handle, uint32_t roi_id, uint32_t threshold_id, uint16_t pix_num_threshold, HysteresisSingleConfTypeDef hysteresis_conf);

/**
 * @brief	get device support ROI number and threshold number
 * @param[in]	handle->DeviceAddr     device address
 * @param[in]   handle->DeviceFd       device fd
 * @param[out]	roi_number      	   
 * @param[out]	threshold_number 	  
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetNumberOfROI(HPS3D_HandleTypeDef *handle, uint8_t *roi_number, uint8_t *threshold_number);

/**
 * @brief	get ROI parameter
 * @param[in]	handle->DeviceAddr     									device address
 * @param[in]   handle->DeviceFd            							device fd
 * @param[in]   roi_id  		       									
 * @param[out]  roi_conf			   									
 * @param[out]	roi_conf.roi_id											
 * @param[out]	roi_conf.enable											
 * @param[out]	roi_conf.left_top_x										
 * @param[out]	roi_conf.left_top_y										
 * @param[out]	roi_conf.right_bottom_x									
 * @param[out]	roi_conf.right_bottom_y									
 *
 * （threshold  output parameter）
 * @param[out]	roi_conf.ref_type[threshold_id]							ROI reference type
 * @param[out]	roi_conf.alarm_type[threshold_id]						ROI alarm type
 * @param[out]	roi_conf.pixel_number_threshold[threshold_id]			
 * @param[out]	roi_conf.hysteresis_conf[threshold_id].threshold_id		
 * @param[out]	roi_conf.hysteresis_conf[threshold_id].hysteresis		
 * @param[out]	roi_conf.hysteresis_conf[threshold_id].threshold_value	
 * @param[out]	roi_conf.hysteresis_conf[threshold_id].positive			
 * @param[out]	roi_conf.hysteresis_conf[threshold_id].enable		
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetROIConfById(HPS3D_HandleTypeDef *handle, uint8_t roi_id, ROIConfTypeDef *roi_conf);

/**
 * @brief	set GPOUT parameter
 * @param[in]	handle->DeviceAddr       device address
 * @param[in]   handle->DeviceFd         device fd
 * @param[in]	gpio_out_conf 		     
 * @param[in]	gpio_out_conf.gpio	     
 * @param[in]	gpio_out_conf.function   
 * @param[in]	gpio_out_conf.polarity   
 * @note        
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetGPIOOut(HPS3D_HandleTypeDef *handle, GPIOOutConfTypeDef gpio_out_conf);

/**
 * @brief	get GPOUT parameter
 * @param[in]	handle->DeviceAddr       device address
 * @param[in]   handle->DeviceFd         device fd
 * @param[in]	gpio_out_conf->gpio	     gpio pin
 * @param[out]	gpio_out_conf    	    
 * @param[out]	gpio_out_conf.function   
 * @param[out]	gpio_out_conf.polarity   
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetGPIOOutConf(HPS3D_HandleTypeDef *handle, GPIOOutConfTypeDef *gpio_out_conf);

/**
 * @brief	set GPIN parameter
 * @param[in]	handle->DeviceAddr      device address
 * @param[in]   handle->DeviceFd        device fd
 * @param[in]	gpio_in_conf 		    
 * @param[in]	gpio_in_conf.gpio	    gpio pin
 * @param[in]	gpio_in_conf.function  
 * @param[in]	gpio_in_conf.polarity   
 * @note		
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetGPIOIn(HPS3D_HandleTypeDef *handle, GPIOInConfTypeDef gpio_in_conf);

/**
 * @brief	get GPIN parameter
 * @param[in]	handle->DeviceAddr      device address
 * @param[in]   handle->DeviceFd        device fd
 * @param[in]	gpio		  		    gpio pin
 * @param[out]	gpio_in_conf   	  	    
 * @param[out]	gpio_in_conf.function  
 * @param[out]	gpio_in_conf.polarity   
 * @note
 * @retval	 RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetGPIOInConf(HPS3D_HandleTypeDef *handle, GPIOInConfTypeDef *gpio_in_conf);

/**
 * @brief	setHDR mode
 * @param[in]	handle->DeviceAddr     	device address
 * @param[in]   handle->DeviceFd        device fd
 * @param[in]	hdr_mode   		 		
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetHDRMode(HPS3D_HandleTypeDef *handle, HDRModeTypeDef hdr_mode);

/**
 * @brief	setHDR
 * @param[in]	handle->DeviceAddr     					device address
 * @param[in]   handle->DeviceFd        				device fd
 * @param[in]	hdr_conf    		   					
 * @param[in]	hdr_conf.hdr_mode						
 * 1、AUTO-HDR parameter：
 * @param[in]	hdr_conf.qualtity_overexposed			
 * @param[in]	hdr_conf.qualtity_overexposed_serious	
 * @param[in]	hdr_conf.qualtity_weak					
 * @param[in]	hdr_conf.qualtity_weak_serious			
 * 2、SIMPLE-HDR parameter：：
 * @param[in]	hdr_conf.simple_hdr_max_integration		
 * @param[in]	hdr_conf.simple_hdr_min_integration		
 * 3、SUPER-HDR parameter：
 * @param[in]	hdr_conf.super_hdr_frame_number			
 * @param[in]	hdr_conf.super_hdr_max_integration		
 * 4、HDR-DISABLE parameter：
 * @param[in]	hdr_conf.hdr_disable_integration_time
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetHDRConfig(HPS3D_HandleTypeDef *handle, HDRConf hdr_conf);

/**
 * @brief	get HDR parameter
 * @param[in]	handle->DeviceAddr     					device address
 * @param[in]   handle->DeviceFd        				device fd
 * @param[out]	hdr_conf	  		   					
 * @param[out]	hdr_conf.hdr_mode						
 * @param[out]	hdr_conf.qualtity_overexposed			
 * @param[out]	hdr_conf.qualtity_overexposed_serious	
 * @param[out]	hdr_conf.qualtity_weak					
 * @param[out]	hdr_conf.qualtity_weak_serious			
 * @param[out]	hdr_conf.simple_hdr_max_integration		
 * @param[out]	hdr_conf.simple_hdr_min_integration		
 * @param[out]	hdr_conf.super_hdr_frame_number			
 * @param[out]	hdr_conf.super_hdr_max_integration		
 * @param[out]	hdr_conf.hdr_disable_integration_time	
 * @note
 * @retval	 RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetHDRConfig(HPS3D_HandleTypeDef *handle, HDRConf *hdr_conf);

/**
 * @brief	set distance filter parameter
 * @param[in]	handle->DeviceAddr     	  		device address
 * @param[in]   handle->DeviceFd          		device fd
 * @param[in]	distance_filter_conf 	  		filter type
 * 				-DISTANCE_FILTER_DISABLE  		
 * 				-DISTANCE_FILTER_SIMPLE_KALMAN	
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetDistanceFilterType(HPS3D_HandleTypeDef *handle, DistanceFilterTypeDef distance_filter_conf);

/**
 * @brief	set Simple kalman parameter
 * @param[in]	handle->DeviceAddr     	  				device address
 * @param[in]   handle->DeviceFd          				device fd
 * @param[in]	distance_filter_conf	  				
 * @param[in]	distance_filter_conf.kalman_K			
 * @param[in]	distance_filter_conf.num_check			
 * @param[in]	distance_filter_conf.kalman_threshold	
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetSimpleKalman(HPS3D_HandleTypeDef *handle, DistanceFilterConfTypeDef distance_filter_conf);

/**
 * @brief	get distance filter parameter
 * @param[in]	handle->DeviceAddr     	  				device address
 * @param[in]   handle->DeviceFd          				device fd
 * @param[out]	distance_filter_conf   	  				
 * @param[out]	distance_filter_conf.filter_type		
 * @param[out]	distance_filter_conf.kalman_K			
 * @param[out]	distance_filter_conf.num_check			
 * @param[out]	distance_filter_conf.kalman_threshold	
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetDistanceFilterConf(HPS3D_HandleTypeDef *handle, DistanceFilterConfTypeDef *distance_filter_conf);

/**
 * @brief	set smooth filter parameter
 * @param[in]	handle->DeviceAddr     	  	device address
 * @param[in]   handle->DeviceFd          	device fd
 * @param[in]	smooth_filter_conf	  	  	
 * @param[in]	smooth_filter_conf.type		
 * @param[in]	smooth_filter_conf.arg1		
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetSmoothFilter(HPS3D_HandleTypeDef *handle, SmoothFilterConfTypeDef smooth_filter_conf);

/**
 * @brief	get smooth filter parameter
 * @param[in]	handle->DeviceAddr     	  	device address
 * @param[in]   handle->DeviceFd          	device fd
 * @param[out]	smooth_filter_conf	  	  	
 * @param[out]	smooth_filter_conf.type		
 * @param[out]	smooth_filter_conf.arg1		
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetSmoothFilterConf(HPS3D_HandleTypeDef *handle, SmoothFilterConfTypeDef *smooth_filter_conf);

/**
 * @brief	Set optical parameter enable or disable
 * @param[in]	handle->DeviceAddr     	  device address
 * @param[in]   handle->DeviceFd          device fd
 * @param[in]	en			  			  
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetOpticalEnable(HPS3D_HandleTypeDef *handle, bool en);

/**
 * @brief	get optical parameter
 * @param[in]	handle->DeviceAddr     	 		 			device address
 * @param[in]   handle->DeviceFd          					device fd
 * @param[out]	optical_param_conf	 	  					
 * @param[out]	optical_param_conf.enable					
 * @param[out]	optical_param_conf.viewing_angle_horiz		
 * @param[out]	optical_param_conf.viewing_angle_vertical	
 * @param[out]	optical_param_conf.illum_angle_horiz		
 * @param[out]	optical_param_conf.illum_angle_vertical		
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetOpticalParamConf(HPS3D_HandleTypeDef *handle, OpticalParamConfTypeDef *optical_param_conf);

/**
* @brief	set distance offset
* @param[in]	handle->DeviceAddr     	  device address
* @param[in]    handle->DeviceFd          device fd
* @param[in]	offset		  			  
* @note
* @retval	RET OK returned successfully
*/
extern RET_StatusTypeDef HPS3D_SetDistanceOffset(HPS3D_HandleTypeDef *handle, int16_t offset);

/**
* @brief	get distance offset
* @param[in]	handle->DeviceAddr     	  device address
* @param[in]    handle->DeviceFd          device fd
* @param[out]	offset		  			  
* @note
* @retval	RET OK returned successfully
*/
extern RET_StatusTypeDef HPS3D_GetDistanceOffset(HPS3D_HandleTypeDef *handle, int16_t *offset);

/**
 * @brief	set interference detect enable
 * @param[in]	handle->DeviceAddr     	  device address
 * @param[in]   handle->DeviceFd          device fd
 * @param[in]	en		  	 			  
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectEn(HPS3D_HandleTypeDef *handle, bool en);

/**
 * @brief	set interference detect integ. time
 * @param[in]	handle->DeviceAddr     	  device address
 * @param[in]   handle->DeviceFd          device fd
 * @param[in]	us		  	  			  
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectIntegTime(HPS3D_HandleTypeDef *handle, uint32_t us);

/**
 * @brief	set interference detect amplitude threshold 
 * @param[in]	handle->DeviceAddr     	  device address
 * @param[in]   handle->DeviceFd          device fd
 * @param[in]	thre		  			  
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectAmplitudeThreshold(HPS3D_HandleTypeDef *handle, uint16_t thre);

/**
 * @brief	set interference detect capture number
 * @param[in]	handle->DeviceAddr     	  device address
 * @param[in]   handle->DeviceFd          device fd
 * @param[in]	capture_number  		 
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectCaptureNumber(HPS3D_HandleTypeDef *handle, uint8_t capture_number);

/**
 * @brief	set interference detect check number
 * @param[in]	handle->DeviceAddr     	  device address
 * @param[in]   handle->DeviceFd          device fd
 * @param[in]	capture_number_check   	 
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetInterferenceDetectNumberCheck(HPS3D_HandleTypeDef *handle, uint8_t capture_number_check);

/**
 * @brief	get interference detect parameter
 * @param[in]	handle->DeviceAddr     	  						device address
 * @param[in]   handle->DeviceFd          						device fd
 * @param[out]	interference_detect_conf  			
 * @param[out]	interference_detect_conf.enable				
 * @param[out]	interference_detect_conf.integ_time				
 * @param[out]	interference_detect_conf.amplitude_threshold	
 * @param[out]	interference_detect_conf.capture_num			
 * @param[out]	interference_detect_conf.number_check			
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetInterferenceDetectConf(HPS3D_HandleTypeDef *handle, InterferenceDetectConfTypeDef *interference_detect_conf);

/**
 * @brief	set assemble angle enable
 * @param[in]	handle->DeviceAddr     	  device address
 * @param[in]   handle->DeviceFd          device fd
 * @param[in]	en		      			 
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetMountingAngleEnable(HPS3D_HandleTypeDef *handle, bool en);

/**
 * @brief	set assemble angle parameter
 * @param[in]	handle->DeviceAddr     	  					device address
 * @param[in]   handle->DeviceFd          					device fd
 * @param[in]	mounting_angle_param_conf 					
 * @param[in]	mounting_angle_param_conf.angle_vertical	
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetMountingAngleParamConf(HPS3D_HandleTypeDef *handle, MountingAngleParamTypeDef mounting_angle_param_conf);

/**
 * @brief	get assemble angle parameter
 * @param[in]	handle->DeviceAddr     	  					device address
 * @param[in]   handle->DeviceFd          					device fd
 * @param[out]	mounting_angle_param_conf 					
 * @param[out]	mounting_angle_param_conf.enable			
 * @param[out]	mounting_angle_param_conf.angle_vertical	
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_GetMountingParamConf(HPS3D_HandleTypeDef *handle, MountingAngleParamTypeDef *mounting_angle_param_conf);



/*************************************2.Integrated function interface**********************************/

/**
 * @brief	get Directory to specify the prefix file
 * @param[in]	dirPath Device file root directory
 * @param[in]   prefix  Device file prefix
 * @param[out]  fileName Used to save devices found in the current directory
 * @note		eg：n = HPS3D_GetDeviceList("/dev/","ttyACM",fileName);
 * @retval		Returns the number of successes
 */
extern uint32_t HPS3D_GetDeviceList(uint8_t * dirPath,uint8_t *prefix,uint8_t fileName[DEV_NUM][DEV_NAME_SIZE]);

/**
 * @brief	Device connect
 * @param[in]	handle->DeviceName  	device file name
 * @param[out]	handle->DeviceFd		device fd
 * @param[out]	handle->ConnectStatus	set ConnectStatus = true;
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_Connect(HPS3D_HandleTypeDef *handle);

/**
 * @brief	Device disConnect 
 * @param[in]	handle->DeviceFd		device fd
 * @param[out]	handle->ConnectStatus	set ConnectStatus = false;
 * @note	
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_DisConnect(HPS3D_HandleTypeDef *handle);


/**
 * @brief		Device initialization
 * @param[in]	handle->mode				run mode 
 * @param[in]	handle->SyncMode			sync mode 
 * @param[out]	handle->DeviceAddr			device address
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_ConfigInit(HPS3D_HandleTypeDef *handle);


/**
 * @brief	set ROI threshold parameter
 * @param[in]	handle->DeviceAddr     									device address
 * @param[in]	handle->DeviceFd										device fd
 * @param[in]	threshold_id		  									
 * @param[in]	roi_conf			   									
 * @param[in]	roi_conf.roi_id											
 * @param[in]	roi_conf.ref_type[threshold_id]							
 * @param[in]	roi_conf.alarm_type[threshold_id]						
 * @param[in]	roi_conf.pixel_number_threshold[threshold_id]			
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].hysteresis		
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].threshold_value	
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].positive			
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].enable			
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetThreshold(HPS3D_HandleTypeDef *handle, uint8_t threshold_id, ROIConfTypeDef roi_conf);

/**
 * @brief	set single ROI parameter
 * @param[in]	handle->DeviceAddr     									device address
 * @param[in]	handle->DeviceFd										device fd
 * @param[in]   roi_conf 			   									
 * @param[in]	roi_conf.roi_id											
 * @param[in]	roi_conf.enable											
 * @param[in]	roi_conf.left_top_x										
 * @param[in]	roi_conf.left_top_y										
 * @param[in]	roi_conf.right_bottom_x									
 * @param[in]	roi_conf.right_bottom_y									
 *
 * set threshold parameter
 * @param[in]	roi_conf.ref_type[threshold_id]							ROI referencr type
 * @param[in]	roi_conf.alarm_type[threshold_id]						ROI alarm type
 * @param[in]	roi_conf.pixel_number_threshold[threshold_id]			
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].roi_id			
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].hysteresis		
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].threshold_value	
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].positive			
 * @param[in]	roi_conf.hysteresis_conf[threshold_id].enable			
 *
 * @param[in]   gpio_out_conf		  	 								
 * @param[in]	gpio_out_conf.gpio	   								
 * @param[in]	gpio_out_conf.function 								
 * @param[in]	gpio_out_conf.polarity 								
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetSingleRoi(HPS3D_HandleTypeDef *handle, ROIConfTypeDef roi_conf, GPIOOutConfTypeDef gpio_out_conf);

/**
 * @brief	Asynchronous mode (continuous measurement mode) adds observers
 * @param[in]	Observer_t               		
 * @param[in]	Observer_t->ObserverFunAddr     
 * @param[in]   Observer_t->NotifyEnable 		
 * @param[in]   Observer_t->AsyncEvent  	
 * @param[in]   Observer_t->ObserverID   		
 * @note Memory must be freed by calling the remove observer function when exiting observer mode
 * @retval RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_AddObserver(void * (*fun)(HPS3D_HandleTypeDef *,AsyncIObserver_t *),HPS3D_HandleTypeDef *handle,AsyncIObserver_t *Observer_t);

/**
 * @brief	Asynchronous mode (continuous measurement mode) removes the observer
 * @param[in]	Observer_t             		  
 * @param[in]	Observer_t->ObserverFunAddr   
 * @param[in]   Observer_t->ObserverID 		 
 * @param[in]   Observer_t->AsyncEvent 		  
 * @note
 * @retval RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_RemoveObserver(AsyncIObserver_t *Observer_t);

/**
 * @brief	Device uninstall and resource recovery (thread exit and resource release)
 * @param[in]	handle->DeviceAddr     device address
 * @retval RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_RemoveDevice(HPS3D_HandleTypeDef *handle);

/**
 * @brief		set debug eanble
 * @param[in]	en	default false
 * @note
 * @retval	 RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetDebugEnable(bool en);

/**
 * @brief		get debug enable
 * @param
 * @note
 * @retval	 true or false
 */
extern bool HPS3D_GetDebugEnable(void);

/**
 * @brief		Receives the address of the callback function
 * @param[in]	void *Call_Back :The address of the receiving callback function is void *fun(uint8_t *str, uint16_t *str_len){...}
 * @param[out]  Returns to the callback functions STR and strlen
 * @note
 * @retval	 RET OK returned successfully
 */
RET_StatusTypeDef HPS3D_SetDebugFunc(void (*Call_Back)(uint8_t *str));

/**
 * @brief	set Point cloud data conversion enabled
 * @param[in]	en    
 * @note
 * @retval	RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SetPointCloudEn(bool en);

/**
 * @brief	get Point cloud data conversion enabled
 * @param
 * @note
 * @retval	true or false
 */
extern bool HPS3D_GetPointCloudEn(void);

/**
 * @brief		Convert to point cloud data
 * @param[in]	MeasureData.full_roi_data/MeasureData.full_depth_data		
 * @param[in]	RetPacketType												
 * @param[out]  MeasureData.point_cloud_data				
 * @note
 * @see
 * @code
 *
 * @retval	 RET OK returned successfully
 */
RET_StatusTypeDef HPS3D_BaseToPointCloud(MeasureDataTypeDef *MeasureData, RetPacketTypedef RetPacketType);

/**
 * @brief		Converting to point cloud data concatenates all rois into a 160x60 depth map
 * @param[in]	MeasureData			
 * @param[in]   RetPacketType       The type of measurement data must be FULL_ROI_PACKET
 * @param[out]	distance     		The spliced depth figure is 160x60
 * @note
 * @retval	 RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_RoiDepthCompound(MeasureDataTypeDef *MeasureData,RetPacketTypedef RetPacketType, uint16_t *distance);


/**
 * @brief		single measurement 
 * @param[in]	handle->DeviceAddr		device address
 * @param[out]	handle->RetPacketType	return packet type
 * @param[out]	handle->MeasureData		Measurement data to be saved
 * @note   The method is synchronous measurement, that is, the measurement return value 
 *		   is obtained immediately after calling this function
 * @see
 * @code
 *
 * @retval	 RET OK returned successfully
 */
extern RET_StatusTypeDef HPS3D_SingleMeasurement(HPS3D_HandleTypeDef *handle);

/**
 * @brief		Initialize obstacle extraction parameters
 * @param[in]	Conf ：Obstacle configuration parameters
 * @note
 * @see
 * @code
 *
 * @retval	 RET OK returned successfully
 */
extern ObstacleConfigTypedef HPS3D_GetObstacleConfigInit(void);

/**
 * @brief		get obstacle extraction parameters
 * @param
 * @note
 * @see
 * @code
 * @retval	 Return obstacle configuration parameter information
 */
extern RET_StatusTypeDef HPS3D_ObstacleConfigInit(ObstacleConfigTypedef *Conf);

/**
 * @brief	get SDK Version
 * @param
 * @note
 * @see
 * @code
 * @retval	return version info.
 */
extern Version_t HPS3D_GetSDKVersion(void);

#ifdef __cplusplus
}
#endif

#endif /* API_H_ */
