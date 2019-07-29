#include<beego_teleop.h>

beego_teleop::beego_teleop()
	:seq(0),linear(0), angular(0)
{
    //publisher
	pub = nhPub.advertise<geometry_msgs::TwistStamped>("/beego/cmd_vel",1);
	//launchファイルからパラメータの読み込み
	setLaunchParam();
	//rqt_reconfigure
	f = boost::bind(&beego_teleop::configCallback, this, _1, _2);
	server.setCallback(f);
}
beego_teleop::~beego_teleop(){
}