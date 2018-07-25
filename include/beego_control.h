#ifndef INCLUDE_BEEGO_CONTROL_CLASS
#define INCLUDE_BEEGO_CONTROL_CLASS

#include"ros/ros.h"
#include <ros/callback_queue.h>
//for image processing on ros
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
//geometry msg
#include<geometry_msgs/Twist.h>
//std_msgs
#include"std_msgs/Int32.h"
#include"std_msgs/Float32.h"
//opencv libraries
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
//beego libraries
#include<headers.h>
#include<setting_parameters.h>
//#include<control_functions.h>

class beego_control{
	private:
		// Variables
		bool gbTorque = false;								// Motor Torque Flag
		bool gbTerminate = false;							// loop flag
		//
		char	sPort[MAX_PORT_NAME];					// Port Name
		
		//oreder message
		geometry_msgs::Twist order_msg;
		//convered oreder message
		int vel_l;
		int vel_r;
		//value of encorder
		int st;
		int fet;//temperature of FET
		float bt;//Motor Driver Battery Voltage
		int vol;//Current Voltage
		int spd;//speed
		int rpc;//RPC(?)
		//ros 
		ros::NodeHandle nh_pub,nh_sub;
		ros::Publisher pub;
		ros::Publisher pub_st;
		ros::Publisher pub_fet;
		ros::Publisher pub_bt;
		ros::Publisher pub_vol;
		ros::Publisher pub_spd;
		ros::Publisher pub_rpc;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
		//beego parameters
		//
		HANDLE	hCommMotor;								// Motor Driver Handle;
		//char	sPort[MAX_PORT_NAME];					// Port Name
		int		iDriveSpeedRight = 0;					// Right Motor Speed
		int		iDriveSpeedLeft = 0;					// Left Motor Speed
		struct timeval	times, timec;					// for Timer
		long	lElapsedTime = 0;						// Elapsed Time
		long	lOldTime = 0;							// Old Time
		int32_t	iMoveStep = 0;							// Moving Step
		int32_t iPosition = 0;							// Right Motor Encoder Count;
		//for find data of acc 
		std::string rcvdata;
	public:

		beego_control();
		~beego_control();
		//subscribe a order 
		void sub_order_vel(void);
		void order_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
		//setup robot
		int setup_robot(void);
		//convert order velocity
		void convert_ordger_vel(void);
		//control the robot
		void control_robot(void);
		//set all encorder values
		void set_encorders(void);
		//set each encorder values
		void set_st(int& pst);
		void set_fet(int& pfet);
		void set_bt(float pbt);
		void set_vol(int& pvol);
		void set_spd(int& pspd);
		void set_rpc(int& pspc);
		//publish encorder values
		void publish_encorders(void);
		
		//beego functions
		int MDR8SendRead(HANDLE hComm, char *psend, int sendlen, char *prcv, int rcvlen, int timeout);
		bool MDR8SendCommand(HANDLE hComm, char *psend, int sendlen, char *prcv, int rcvlen, int timeout);
		bool MDR8MspSend(HANDLE hComm, int drv, int motor, int maxspeed);
		bool MDR8SocSend(HANDLE hComm, int drv, int motor, int current, int ctime);
		bool MDR8RacSend(HANDLE hComm, int drv, int motor);
		bool MDR8MotorControl(HANDLE hComm, int mode, int ctrl, int drv, int motor, int val);
		int MDR8GetStatusLong(HANDLE hComm, int drv, int motor, int *pst, int *pfet, int *pbt, int *pvol, int *pspd, int *prpc);
		bool MDR8BcpSend(HANDLE hComm);
		void setttyportname(int num);
		long getMsecTime(struct timeval *pold, struct timeval *pnew);
		
		
};


#endif
