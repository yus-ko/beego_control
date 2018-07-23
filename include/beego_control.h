#ifndef INCLUDE_BEEGO_CONTROL_CLASS
#define INCLUDE_BEEGO_CONTROL_CLASS

#include"ros/ros.h"
#include <ros/callback_queue.h>
//for image processing on ros
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
//opencv library
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>


class beego_control{
	private:
		//oreder message
		geometry_msgs::Twist order_msg;
		//convered oreder message
		int vel_l;
		int vel_r;
		//value of encorder
		int st;
		int fet;//temperature of FET
		int bt;//Motor Driver Battery Voltage
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
		//
	public:

		beego_control();
		~beego_control();
		//subscribe a order 
		void sub_order_vel(void);
		void order_vel_callback(const geometry_msgs::TwistConstPtr& msg);
		//setup robot
		void setup_robot(void);
		
		//set all encorder values
		void set_encorders(void);
		//set each encorder values
		void set_st(int& pst);
		void set_fet(int& pfet);
		void set_bt(int& pbt);
		void set_vol(int& pvol);
		void set_spd(int& pspd);
		void set_rpc(int& pspc);
		//publish encorder values
		void publish_encorders(void);
		
};

beego_control::beego_control()
	:fet(-1),bt(-1),vol(-1),spd(-1),rpc(-1)
{
	std::cout<<"in beego_control constracter\n";
	//case 1 : publish each value
	pub_st=nh_pub.advertise<std_msgs::int32>("st",1);
	pub_fet=nh_pub.advertise<std_msgs::int32>("temperature_fet",1);
	pub_bt=nh_pub.advertise<std_msgs::float32>("moter_driver_battery",1);
	pub_vol=nh_pub.advertise<std_msgs::int32>("current voltage",1);
	pub_rpc=nh_pub.advertise<std_msgs::int32>("rcp",1);
	//case 2 : publish a value included each value
	//pub=nh_pub.advertise<beego_control::encorder>("encorder",1);
	//------------------------------------------------------
	
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/cmd_vel",1,&beego_control::order_vel_callback,this);

}
void beego_control::sub_order_vel(void)
{
	queue.callOne(ros::WallDuration(1));
}
void beego_control::order_vel_callback(const geometry_msgs::TwistCon::stPtr& msg)
{
	order_msg=msg;
}
beego_control::~beego_control()
{
}
void beego_control::setup_robot(void)
{
	printf("\nProgram Start ....\n");
	gbTorque = false;
	gbTerminate = false;
	
	// Open Motor Driver
	setttyportname(MOTOR_DRIVER_PORT);
	hCommMotor = SerialOpen(sPort, MOTOR_DRIVER_BAUDRATE);
	if(hCommMotor <= 0){
		printf("Motor Driver not found.[ERROR]\n");
		return -1;
	} else {
		printf("Motor Driver found.[%d]\n", MOTOR_DRIVER_BAUDRATE);
	}

	// Setup Motor Driver
	// Position Mode Speed = 50
	if(!MDR8MspSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 3000)){
		printf("Motor Driver Msp Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_RIGHT);
		SerialClose(hCommMotor);
		return -1;
	}
	if(!MDR8MspSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 3000)){
		printf("Motor Driver Msp Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_LEFT);
		SerialClose(hCommMotor);
		return -1;
	}

	// 5A ... 300 msec
	if(!MDR8SocSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 5000, 300)){
		printf("Motor Driver Soc Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_RIGHT);
		SerialClose(hCommMotor);
		return -1;
	}
	if(!MDR8SocSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 5000, 300)){
		printf("Motor Driver Soc Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_LEFT);
		SerialClose(hCommMotor);
		return -1;
	}

	// Reset Counter
	if(!MDR8RacSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT)){
		printf("Motor Driver Rac Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_RIGHT);
		SerialClose(hCommMotor);
		return -1;
	}
	if(!MDR8RacSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT)){
		printf("Motor Driver Rac Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_LEFT);
		SerialClose(hCommMotor);
		return -1;
	}

	// Torque Off
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_TORQUE_OFF, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 0);
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_TORQUE_OFF, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 0);
	// Speed = 0
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 0);
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 0);

	printf("Start Loop ....\n");
	printf("STOP\n");

	// main loop
	gettimeofday(&times, NULL);							// Set Start time
}
void beego_control::convert_ordger_vel(void)
{
	double v=order_msg.linear.x;//速度
	double w=order_msg.angular.z;//角速度
	//計算式でv,wを車輪速度vel_l,vel_rに変換
	//
	//変換プロセス
	//
	//vel_l=?;//左車輪
	//vel_r=?;//右車輪
}
void beego_control::control_robot(void)
{
	//vel_l=?;//左車輪
	//vel_r=?;//右車輪
	
}
//set encorder values
void beego_control::set_encorders(void)
{
	int st = 0, fet = 0, bt = 0, vol = 0, spd = 0, rpc = 0;
		
	MDR8GetStatusLong(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, &st, &fet, &bt, &vol, &spd, &rpc);
	printf("%.1fV L:%d ", (float)bt/10.0, rpc);
	MDR8GetStatusLong(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, &st, &fet, &bt, &vol, &spd, &rpc);
	printf("R:%d\n", rpc);
	
	//set encorders
	set_st(st);
	set_fet(fet);
	set_bt((float)bt/10.0);
	set_vol(vol);
	set_spd(spd);
	set_rpc(rpc);
}
void beego_control::set_st(int& pst)
{
	st=pst;
}
void beego_control::set_fet(int& pfet)
{
	st=pfet;
}
void beego_control::set_bt(float pbt)
{
	bt=pbt;
}
void beego_control::set_vol(int& pvol)
{
	vol=pvol;
}
void beego_control::set_spd(int& pspd)
{
	spd=pspd;
}
void beego_control::set_rpc(int& prpc)
{
	rpc=prpc;
}


//publish encorder values
void beego_control::publish_encorders(void)
{
	//value of encorder(ros msgs)
	std_msgs::int32 msg_st;
	std_msgs::int32 msg_fet;//temperature of FET
	std_msgs::float32 msg_bt;//Motor Driver Battery Voltage
	std_msgs::int32 msg_vol;//Current Voltage
	std_msgs::int32 msg_spd;//speed
	std_msgs::int32 msg_rpc;//RPC(?)
	//set messages
	msg_st=st;
	msg_fet=fet;
	msg_bt=bt;
	msg_vol=vol;
	msg_spd=spd;
	msg_rpc=rpc;
	//publish messagegs
	pub_st.publish(msg_st);
	pub_fet.publish(msg_fet);
	pub_bt.publish(msg_bt);
	pub_vol.publish(msg_vol);
	pub_spd.publish(msg_spd);
	pub_rpc.publish(msg_rpc);
	
}

#endif
