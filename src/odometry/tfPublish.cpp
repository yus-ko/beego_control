#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


void odometry_callback(const nav_msgs::Odometry::ConstPtr& msg){

    ros::Time current_time;
    current_time = ros::Time::now();
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = state_odom_x;
    odom_trans.transform.translation.y = state_odom_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.header.stamp = current_time;
}

void odom_calc(double &state_odom_x,double &state_odom_y,double &state_odom_th){
    //ここでエンコーダ値を読み取り、オドメトリ変換する演算を行う。
}

int main(int argc, char **argv){
    ros::NodeHandle nhSub1;
    ros::Subscriber subOdom,subAcc,subVel;
    tf::TransformBroadcaster odom_broadcaster;
    // ros::Time current_time;
    // ::ClassificationData cd;

    // ros::Rate r(30.0);
    double state_odom_x=0.0;//オドメトリX座標[m]
    double state_odom_y=0.0;//オドメトリY座標[m]
    double state_odom_th=0.0; //オドメトリ姿勢[rad]
    while(nh.ok()){
    odom_calc(state_odom_x,state_odom_y,state_odom_th);
    // current_time = ros::Time::now();

    //tf odom->base_link

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state_odom_th);
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    ros::spinOnce();
    // r.sleep();
    }
}