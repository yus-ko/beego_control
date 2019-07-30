#include<beego_control.h>

beego::control::control()
	:fet(-1),bt(-1),vol(-1),leftspd(-1),leftrpc(-1),rightspd(-1),rightrpc(-1),seq(0),
	sample_time_p(-1),sample_time_c(-1),sample_time_delta(-1)
{
	std::cout<<"in beego_control constracter\n";
	pub=nh_pub.advertise<beego_control::beego_encoder>("encoder",1);
	//------------------------------------------------------
	
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/beego/cmd_vel",1,&beego::control::order_vel_callback,this);
	order_msg.linear.x=0;
	order_msg.linear.y=0;
	order_msg.linear.z=0;
	order_msg.angular.x=0;
	order_msg.angular.y=0;
	order_msg.angular.z=0;
	
}
void beego::control::sub_order_vel(void)
{
	queue.callOne(ros::WallDuration(0.01));
}
void beego::control::order_vel_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	order_msg.linear=msg->twist.linear;
	order_msg.angular=msg->twist.angular;
}
beego::control::~control()
{
}
int beego::control::setup_robot(void)
{
	printf("\nProgram Start ....\n");
	gbTorque = false;
	gbTerminate = false;
	
	// Open Motor Driver
	setttyportname(MOTOR_DRIVER_PORT);
	hCommMotor = SerialOpen(sPort, MOTOR_DRIVER_BAUDRATE);
	if(hCommMotor <= 0){
		printf("Motor Driver not found.[ERROR]\n");
		return -1;
	} else {
		printf("Motor Driver found.[%d]\n", MOTOR_DRIVER_BAUDRATE);
	}

	// Setup Motor Driver
	// Position Mode Speed = 50
	if(!MDR8MspSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 3000)){
		printf("Motor Driver Msp Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_RIGHT);
		SerialClose(hCommMotor);
		return -1;
	}
	if(!MDR8MspSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 3000)){
		printf("Motor Driver Msp Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_LEFT);
		SerialClose(hCommMotor);
		return -1;
	}

	// 5A ... 300 msec
	if(!MDR8SocSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 5000, 300)){
		printf("Motor Driver Soc Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_RIGHT);
		SerialClose(hCommMotor);
		return -1;
	}
	if(!MDR8SocSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 5000, 300)){
		printf("Motor Driver Soc Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_LEFT);
		SerialClose(hCommMotor);
		return -1;
	}

	// Reset Counter
	if(!MDR8RacSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT)){
		printf("Motor Driver Rac Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_RIGHT);
		SerialClose(hCommMotor);
		return -1;
	}
	if(!MDR8RacSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT)){
		printf("Motor Driver Rac Error %d-%d.[ERROR]\n", MOTOR_DRIVER_NUMBER, MOTOR_LEFT);
		SerialClose(hCommMotor);
		return -1;
	}

	// Torque Off
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_TORQUE_OFF, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 0);
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_TORQUE_OFF, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 0);
	// Speed = 0
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 0);
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 0);

	printf("Start Loop ....\n");
	printf("STOP\n");

	// main loop
	gettimeofday(&times, NULL);							// Set Start time
	//
	printf("Torque ON\n");
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_TORQUE_ON, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 0);
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_TORQUE_ON, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 0);

}
void beego::control::convert_ordger_vel(void)
{
	double v=order_msg.linear.x;//速度
	double w=order_msg.angular.z;//角速度
	double d=0.14;//車幅の半分の長さ
	//計算式でv,wを車輪速度vel_l,vel_rに変換
	double vl=v+d*w;//左速度
	double vr=v-d*w;//右速度
	//変換プロセス
	double a=3.1416*0.082;
	double b=26.85*45*60/16;
	vel_l=vl*b/a;//左車輪
	vel_r=vr*b/a;//右車輪
	std::cout<<"vel_l,vel_r:"<<vel_l<<","<<vel_r<<"\n";
}
void beego::control::control_robot(void)
{
	//vel_l=?;//左車輪
	//vel_r=?;//右車輪
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, -vel_r);
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, vel_l);
	MDR8BcpSend(hCommMotor);
	
	//gettimeofday(&times, NULL);
	
}
//set encoder values
bool beego::control::set_encoders(void)
{
	int st = 0, fet = 0, bt = 0, vol = 0, spd = 0, rpc = 0;
		
	MDR8GetStatusLong(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, &st, &fet, &bt, &vol, &spd, &rpc);
	printf("%.1fV L:%d ", (float)bt/10.0, rpc);
	
	//set encoders
	set_st(st);
	set_fet(fet);
	set_bt((float)bt/10.0);
	set_vol(vol);
	set_leftspd(spd);
	set_leftrpc(rpc);
	
	MDR8GetStatusLong(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, &st, &fet, &bt, &vol, &spd, &rpc);
	printf("R:%d\n", rpc);
	set_rightspd(spd);
	set_rightrpc(rpc);
	if(set_odom()){
		printf("t:%d, vl:%f,vr:%f,dl:%f,dr:%f\n",sample_time_c, leftspd,rightspd, leftodom, rightodom);
		return true;
	}
	return false;
	
}
void beego::control::set_st(int& pst)
{
	st=pst;
}
void beego::control::set_fet(int& pfet)
{
	st=pfet;
}
void beego::control::set_bt(float pbt)
{
	bt=pbt;
}
void beego::control::set_vol(int& pvol)
{
	vol=pvol;
}
void beego::control::set_leftspd(int& pspd)
{
	leftspd=pspd*0.00005685591505;
}
void beego::control::set_leftrpc(int& prpc)
{
	leftrpc=prpc;
}
void beego::control::set_rightspd(int& pspd)
{
	rightspd=pspd*0.00005685591505;
}
void beego::control::set_rightrpc(int& prpc)
{
	rightrpc=prpc;
}
bool beego::control::set_odom(void)
{
	if(sample_time_p > 0){
		sample_time_delta = sample_time_c - sample_time_p;
		rightodom=rightspd*(float)sample_time_delta / 1000;
		leftodom=leftspd*(float)sample_time_delta / 1000;
	}
	else{
		return false;
	}
	return true;
}
//set sensor value
void beego::control::set_acc_gyro(void)
{
	int acc_x=0, acc_y=0, acc_z=0, gyro_x=0, gyro_y=0, gyro_z=0;
	
	get_acc_gyro(hCommMotor, MOTOR_DRIVER_NUMBER, &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
	//set encoders
	set_acc_x(acc_x);
	set_acc_y(acc_y);
	set_acc_z(acc_z);
	set_gyro_x(gyro_x);
	set_gyro_y(gyro_y);
	set_gyro_z(gyro_z);
}
void beego::control::set_acc_x(int& pacc_x)
{
	acc_x=pacc_x * 9.807 * 0.01;
}
void beego::control::set_acc_y(int& pacc_y)
{
	acc_y=pacc_y * 9.807 * 0.01;
}
void beego::control::set_acc_z(int& pacc_z)
{
	acc_z=pacc_z * 9.807 * 0.01;
}
void beego::control::set_gyro_x(int& pgyro_x)
{
	gyro_x=pgyro_x;
}
void beego::control::set_gyro_y(int& pgyro_y)
{
	gyro_y=pgyro_y;
}
void beego::control::set_gyro_z(int& pgyro_z)
{
	gyro_z=pgyro_z;
}


//publish encoder values
void beego::control::publish_encoders(void)
{
	//value of encoder(ros msgs)
	
	std::cout<<"gyro "<<atan(acc_y/std::sqrt(acc_x*acc_x+acc_z*acc_z))*180/M_PI<<" "<<atan(acc_x/std::sqrt(acc_y*acc_y+acc_z*acc_z))*180/M_PI<<"\n";

	//encoder msgs
	beego_control::beego_encoder encoder;
	std_msgs::Header header;
	header.frame_id = "base_link";
	header.seq = seq;
	header.stamp = ros_sample_time;
	encoder.header =header;
	encoder.st = st;
	encoder.fet = fet;
	encoder.bt = bt;
	encoder.vol = vol;
	encoder.gyro.x = gyro_x;
	encoder.gyro.y = gyro_y;
	encoder.gyro.z = gyro_z;
	encoder.acc.x = acc_x;
	encoder.acc.y = acc_y;
	encoder.acc.z = acc_z;
	encoder.rpc.l = leftrpc;
	encoder.rpc.r = rightrpc;
	encoder.vel.l = leftspd;
	encoder.vel.r = rightspd;
	pub.publish(encoder);
}




