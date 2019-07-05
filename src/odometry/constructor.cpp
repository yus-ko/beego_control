#include<beego_control/odometry.h>

odometryClass::odometryClass()
	:subOdom(nhSub, "/rcp", 1),subVel(nhSub, "/speed", 1),subAcc(nhSub, "/acc", 1),
	sync(MySyncPolicy(10),subOdom, subAcc, subVel)
{
    //subscriber
    sync.registerCallback(boost::bind(&odometryClass::encorder_callback, this,_1, _2, _3));
	//publisher
    // pub= nhPub.advertise<beego_control::ClassificationVelocityData>("classificationData", 1);

	//launchファイルからパラメータの読み込み
	setLaunchParam();
	//デバッグ用
	//publisher
    // pubDeb= nhDeb.advertise<sensor_msgs::Image>("windowImage", 1);
    // pubDebPcl= nhDebPcl.advertise<sensor_msgs::PointCloud2>("visualizedCluster", 1);

	//rqt_reconfigure
	f = boost::bind(&odometryClass::configCallback, this, _1, _2);
	server.setCallback(f);
}
odometryClass::~odometryClass(){
}