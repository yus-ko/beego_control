#include<odometry.h>

void odometryClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    geometry_msgs::Point pos;
    geometry_msgs::Quaternion quat;
    double yaw;
    //初期オドメトリ
    // n.getParam("",);
    initOdom.child_frame_id = "base_link";
    //set pose
    pos.x = 0;
    pos.y = 0;
    pos.z = 0;
    yaw = 0;
    d = 0.3145;//仮
	//tf:(x座標, y座標 ) = robot:(y座標, -x座標)
    n.getParam("posX",pos.y);
    pos.y = -pos.y;
    n.getParam("posY",pos.x);
    n.getParam("yaw",yaw);
    n.getParam("d",d);
    //culc Quaternion
    quat = tf::createQuaternionMsgFromYaw(yaw);
    initOdom.pose.pose.position = pos;
    initOdom.pose.pose.orientation = quat;
    geometry_msgs::Twist twist;
    //init twist
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    initOdom.twist.twist = twist;
}