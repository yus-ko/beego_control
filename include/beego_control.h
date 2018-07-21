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
		//value of encorder
		int fet;//temperature of FET
		int mdbv;//Motor Driver Battery Voltage
		int vol;//Current Voltage
		int spd;//speed
		int rpc;//RPC(?)
		//ros 
		ros::NodeHandle nh_pub,nh_sub;
		ros::Publisher pub;
		ros::Publisher pub_fet;
		ros::Publisher pub_mdbv;
		ros::Publisher pub_vol;
		ros::Publisher pub_rpc;
		ros::Subscriber sub;
		ros::CallbackQueue queue;
		
		
	public:

		beego_control();
		~beego_control();
		//subscribe a order 
		void sub_order_vel(void);
		void order_vel_callback(const geometry_msgs::TwistConstPtr& msg);
		//set the order
		//void set_ordet_vel(void);
		//set all encorder values
		void set_encorders(void);
		//set each encorder values
		void set_fet(void);
		void set_mdbv(void);
		void set_vod(void);
		void set_spd(void);
		void set_rpc(void);
		//publish encorder values
		void publish_encorders(void);
		
};

beego_control::beego_control()
	:fet(-1),mdbv(-1),vol(-1),spd(-1),rpc(-1)
{
	std::cout<<"in beego_control constracter\n";
	//case 1 : publish each value
	pub_fet=nh_pub.advertise<std_msgs::int32>("temperature_fet",1);
	pub_mdbv=nh_pub.advertise<std_msgs::int32>("moter_driver_battery",1);
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
void order_vel_callback(const geometry_msgs::TwistCon::stPtr& msg)
{
	order_msg=msg;
}
beego_control::~beego_control()
{
}
//set encorder values
void beego_control::set_encorders(void);
void beego_control::set_fet(void);
void beego_control::set_mdbv(void);
void beego_control::set_vod(void);
void beego_control::set_spd(void);
void beego_control::set_rpc(void);


//publish encorder values
void beego_control::publish_encorders(void);

#endif
