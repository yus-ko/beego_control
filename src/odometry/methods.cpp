#include<odometry.h>

//subscribe
void odometryClass::encoder_callback(const beego_control::beego_encoder::ConstPtr& msg){
    //データをコピー
	encoder = *msg;
	//move manage method
	manage();
}
void odometryClass::configCallback(beego_control::odometryConfig &config, uint32_t level) {
	ROS_INFO("Reconfigure Request: %f %f %f", 
		config.initPosX, config.initPosY,
		config.initRotYaw
		);
    geometry_msgs::Point pos;
    geometry_msgs::Quaternion quat;
    double yaw;
    //set pose
	//tf:(x座標, y座標 ) = robot:(y座標, -x座標)
    pos.x = config.initPosY;
    pos.y = -config.initPosX;
    pos.z = 0;
    yaw = config.initRotYaw;
	//culc Quaternion
    quat = tf::createQuaternionMsgFromYaw(yaw);
	//set odometry
    initOdom.pose.pose.position = pos;
    initOdom.pose.pose.orientation = quat;
}
void odometryClass::culcOdometry(){
	//仮変数
	float d;
	//
    geometry_msgs::Point pos = difOdom.pose.pose.position;
    geometry_msgs::Quaternion quat = difOdom.pose.pose.orientation;
	tf::Quaternion quatTf;
	double roll, pitch, yaw;
    quaternionMsgToTF(quat, quatTf);
    tf::Matrix3x3(quatTf).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
	//
	//Lr,Ll : 車輪位置の偏差
	//length: 位置の偏差, angle: 角度の偏差
	//rpc -> Lr,Ll
	//--process
	//...
	//
	//Lr,Ll -> length, angle
	float Lr,Ll;
	float length = (Lr + Ll) / 2.0;
	float angle = (Lr - Ll) / (2.0*d);
	//x(t) = x(t-1) + dL * cos ( theta(t-1) + d_theta/2.0)
	//y(t) = y(t-1) + dL * sin ( theta(t-1) + d_theta/2.0)
	//tf:(x座標, y座標 ) = robot:(y座標, -x座標)
	//
	pos.y = pos.y - length * sin( yaw + angle/2.0);//robot:-x
	pos.x = pos.x + length * cos( yaw + angle/2.0);//robot:y
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
	double yaw = 0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
    odom_trans.transform.rotation = odom_quat;
}
void odometryClass::manage(){
	setOdometry();
	sendTransform();
}
void odometryClass::sendTransform(){//データ送信
    odom_broadcaster.sendTransform(odom_trans);
}
