#include<beego_teleop.h>

void beego_teleop::configCallback(beego_control::teleopConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %d %f %f", 
		config.controlable, config.linear,
		config.angular
		);
	controlable = config.controlable;
	linear = config.linear;
	angular = config.angular;
}
void beego_teleop::manage(){
	setTwistMsgs();
	publishVelocity();
}
void beego_teleop::setTwistMsgs(){
	std_msgs::Header header;
	header.frame_id = "base_link";
	header.seq = seq++;
	header.stamp = ros::Time::now();
	twist.header = header;
	twist.twist.linear.y = 0;
	twist.twist.linear.z = 0;
	twist.twist.angular.x = 0;
	twist.twist.angular.y = 0;
	if(controlable){
		twist.twist.linear.x = linear;
		twist.twist.angular.z = angular;
		ROS_INFO("Velocity(linear,angular):(%f,%f)", linear, angular);
	}
	else{
		twist.twist.linear.x = 0;
		twist.twist.angular.z = 0;
		ROS_INFO("Not controlable!");
	}
}

void beego_teleop::publishVelocity(){//データ送信
	pub.publish(twist);
}
