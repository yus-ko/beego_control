#include<odometry.h>

//subscribe
void odometryClass::encorder_callback(const beego_control::rpc::ConstPtr& rpcMsg, const beego_control::vel::ConstPtr& velMsg, const std_msgs::Int32::ConstPtr& accMsg){
    //データをコピー
	rpc = *rpcMsg;
	vel = *velMsg;
	acc = *accMsg;
	//move manage method
	manage();
}
void odometryClass::configCallback(beego_control::odometryConfig &config, uint32_t level) {
	// ROS_INFO("Reconfigure Request: %d %f %f %d", 
	// 	config.windowDivisionDegree, config.windowHeight,
	// 	config.windowWidth,config.windowMinPts
	// 	// config.str_param.c_str(), 
	// 	// config.bool_param?"True":"False", 
	// 	// config.size
	// 	);

}
void odometryClass::setOdometry(){
    ros::Time current_time;
    current_time = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    //--estimate process
	odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
	//--
    odom_trans.header.stamp = current_time;
	double state_odom_th=0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state_odom_th);
    odom_trans.transform.rotation = odom_quat;
}
void odometryClass::manage(){
	setOdometry();
	sendTransform();
}
void odometryClass::sendTransform(){//データ送信
    odom_broadcaster.sendTransform(odom_trans);
}
