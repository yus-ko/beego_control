#include<beego_control.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"beego_control");
	
	beego_control bc;
	if(bc.setup_robot()==-1)
	{
		ROS_INFO("error");
		return -1;
	}
	while(ros::ok())
	{
		bc.sub_order_vel();	
		bc.convert_ordger_vel();
		bc.control_robot();
		bc.set_encorders();
		bc.set_acc_gyro();
		bc.publish_encorders();
	}
	
	return 0;
}
