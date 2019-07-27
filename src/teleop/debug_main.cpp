#include <beego_teleop.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"beego_control_teleop");
	
    beego_teleop tp; //
    ros::Rate rate(50);
    while(ros::ok()){
        ros::spinOnce();
        tp.manage();
        rate.sleep();
    }
	return 0;
}