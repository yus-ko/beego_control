#ifndef INCLUDE_BEEGO_TELEOP_CLASS
#define INCLUDE_BEEGO_TELEOP_CLASS

#include"ros/ros.h"
#include <ros/callback_queue.h>
//geometry msg
#include<geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>
#include <beego_control/teleopConfig.h>

class beego_teleop{
	private:
		//ros 
		ros::NodeHandle nhPub;
		ros::Publisher pub;
        //--rqt_reconfigure
        dynamic_reconfigure::Server<beego_control::teleopConfig> server;
        dynamic_reconfigure::Server<beego_control::teleopConfig>::CallbackType f;
		geometry_msgs::TwistStamped twist;
		int seq;
		bool controlable;
		float linear, angular;
	public:
		beego_teleop();
		~beego_teleop();
		//launch読み込み
		void setLaunchParam();
		//dynamic reconfigure
		void configCallback(beego_control::teleopConfig &config, uint32_t level);
		//manage
		void manage();
		void setTwistMsgs();
		//publish velocity
		void publishVelocity();		
};

#endif
