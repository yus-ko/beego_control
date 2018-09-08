#include<ros/ros.h>
#include<sstream>
#include<geometry_msgs/Twist.h>
#include<time_class.h>//time class

int main(int argc, char** argv)
{
	ros::init(argc,argv,"rotation");
	ros::NodeHandle NH;
	ros::Publisher pub;
	geometry_msgs::Twist msg;
	//time class
	time_class tm_cls;//time classを定義
	
	pub=NH.advertise<geometry_msgs::Twist>("/beego/cmd_vel",1);
	
	msg.linear.x=0;
	msg.linear.y=0;
	msg.linear.z=0;
	msg.angular.x=0;
	msg.angular.y=0;
	msg.angular.z=0;
	
	
	float l_x = msg.linear.x = 0.1;
	float a_z = msg.angular.z = 0.1;
	
	float T = 2 * 3.1416/a_z;
	float r = l_x/a_z;	


	std::cout<<"r="<<r<<"\n";
	std::cout<<"T="<<T<<"\n";
	

	
	
	tm_cls.set_time();//timeを取得(1回目)
	//
	//なんかしらの処理
	//
	tm_cls.set_time();//timeを取得(2回目)
	//
	double dt = tm_cls.get_delta_time();//1回目から２回目までの経過時間
	//
	tm_cls.set_time();//timeを取得(3回目)
	//
	dt = tm_cls.get_delta_time();//2回目から3回目までの経過時間
	//
	int i=0;
	ros::Rate rate(1);
	while(i++<T&&ros::ok())
	{
		pub.publish(msg);
		rate.sleep();	
	}
	
	msg.linear.x=0;
	msg.linear.y=0;
	msg.linear.z=0;
	msg.angular.x=0;
	msg.angular.y=0;
	msg.angular.z=0;
	pub.publish(msg);
	
	
	return 0;
}
