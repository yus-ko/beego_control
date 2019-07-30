#ifndef INCLUDE_BEEGO_CONTROL_CLASS
#define INCLUDE_BEEGO_CONTROL_CLASS

#include"ros/ros.h"
#include <ros/callback_queue.h>
//for image processing on ros
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
//geometry msg
#include<geometry_msgs/TwistStamped.h>
#include<geometry_msgs/Point32.h>
//std_msgs
#include"std_msgs/Int32.h"
#include"std_msgs/Float32.h"
#include<beego_control/vel.h>
#include<beego_control/rpc.h>
//opencv libraries
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
//beego libraries
#include<headers.h>
#include<setting_parameters.h>
//#include<control_functions.h>
//publish mags
#include<beego_control/beego_encoder.h>
namespace beego{

class control{
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
		//value of encoder
		int st;
		int fet;//temperature of FET
		float bt;//Motor Driver Battery Voltage
		int vol;//Current Voltage
		float leftspd;//speed
		float rightspd;
		float leftodom;//delta wheel odometry 
		float rightodom;
		int leftrpc;//RPC(?)
		int rightrpc;
		//value of sensor 
		float acc_x;
		float acc_y;
		float acc_z;
		float gyro_x;
		float gyro_y;
		float gyro_z;
		//sample time
		int sample_time_p;
		int sample_time_c;
		int sample_time_delta;
		ros::Time ros_sample_time;
		int seq;
		//ros 
		ros::NodeHandle nh_pub,nh_sub;
		ros::Publisher pub;
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

		control();
		~control();
		//subscribe a order 
		void sub_order_vel(void);
		void order_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
		//setup robot
		int setup_robot(void);
		//convert order velocity
		void convert_ordger_vel(void);
		//control the robot
		void control_robot(void);
		//set all encoder values
		bool set_encoders(void);
		//set each encoder values
		void set_st(int& pst);
		void set_fet(int& pfet);
		void set_bt(float pbt);
		void set_vol(int& pvol);
		void set_leftspd(int& pspd);
		void set_leftrpc(int& pspc);
		void set_rightspd(int& pspd);
		void set_rightrpc(int& pspc);
		bool set_odom(void);
		//publish encoder values
		void publish_encoders(void);
		//set sensor value
		void set_acc_gyro(void);
		//set each sensor values
		void set_acc_x(int& pacc_x);
		void set_acc_y(int& pacc_y);
		void set_acc_z(int& pacc_z);
		void set_gyro_x(int& pgyro_x);
		void set_gyro_y(int& pgyro_y);
		void set_gyro_z(int& pgyro_z);

		//get acc gyro
		int get_acc_gyro(HANDLE hComm, int drv, int *acc_x, int *acc_y, int *acc_z, int *gyro_x, int *gyro_y, int *gyro_z);
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
}

#endif
