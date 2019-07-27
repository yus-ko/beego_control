#include<odometry.h>

odometryClass::odometryClass()
{
    //subscriber
	sub=nhSub.subscribe("/encoder",1,&odometryClass::encoder_callback,this);
	//launchファイルからパラメータの読み込み
	setLaunchParam();
	//rqt_reconfigure
	f = boost::bind(&odometryClass::configCallback, this, _1, _2);
	server.setCallback(f);
}
odometryClass::~odometryClass(){
}