#include<beego_control.h>

beego_control::beego_control()
	:fet(-1),bt(-1),vol(-1),spd(-1),rpc(-1)
{
	std::cout<<"in beego_control constracter\n";
	//case 1 : publish each value
	pub_st=nh_pub.advertise<std_msgs::Int32>("st",1);
	pub_fet=nh_pub.advertise<std_msgs::Int32>("temperature_fet",1);
	pub_bt=nh_pub.advertise<std_msgs::Float32>("moter_driver_battery",1);
	pub_vol=nh_pub.advertise<std_msgs::Int32>("current_voltage",1);
	pub_spd=nh_pub.advertise<std_msgs::Int32>("speed",1);
	pub_rpc=nh_pub.advertise<std_msgs::Int32>("rcp",1);
	pub_acc=nh_pub.advertise<geometry_msgs::Point32>("acc",1);
	pub_gyro=nh_pub.advertise<geometry_msgs::Point32>("gyro",1);
	//case 2 : publish a value included each value
	//pub=nh_pub.advertise<beego_control::encorder>("encorder",1);
	//------------------------------------------------------
	
	nh_sub.setCallbackQueue(&queue);
	sub=nh_sub.subscribe("/beego/cmd_vel",1,&beego_control::order_vel_callback,this);
	order_msg.linear.x=0;
	order_msg.linear.y=0;
	order_msg.linear.z=0;
	order_msg.angular.x=0;
	order_msg.angular.y=0;
	order_msg.angular.z=0;
	
}
void beego_control::sub_order_vel(void)
{
	queue.callOne(ros::WallDuration(0.01));
}
void beego_control::order_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	order_msg.linear=msg->linear;
	order_msg.angular=msg->angular;
}
beego_control::~beego_control()
{
}
int beego_control::setup_robot(void)
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
void beego_control::convert_ordger_vel(void)
{
	double v=order_msg.linear.x;//速度
	double w=order_msg.angular.z;//角速度
	double d=0.145;//車幅の半分の長さ
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
void beego_control::control_robot(void)
{
	//vel_l=?;//左車輪
	//vel_r=?;//右車輪
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, -vel_r);
	MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, vel_l);
	MDR8BcpSend(hCommMotor);
	
	//gettimeofday(&times, NULL);
	
}
//set encorder values
void beego_control::set_encorders(void)
{
	int st = 0, fet = 0, bt = 0, vol = 0, spd = 0, rpc = 0;
		
	MDR8GetStatusLong(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, &st, &fet, &bt, &vol, &spd, &rpc);
	printf("%.1fV L:%d ", (float)bt/10.0, rpc);
	MDR8GetStatusLong(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, &st, &fet, &bt, &vol, &spd, &rpc);
	printf("R:%d\n", rpc);
	
	//set encorders
	set_st(st);
	set_fet(fet);
	set_bt((float)bt/10.0);
	set_vol(vol);
	set_spd(spd);
	set_rpc(rpc);
}
void beego_control::set_st(int& pst)
{
	st=pst;
}
void beego_control::set_fet(int& pfet)
{
	st=pfet;
}
void beego_control::set_bt(float pbt)
{
	bt=pbt;
}
void beego_control::set_vol(int& pvol)
{
	vol=pvol;
}
void beego_control::set_spd(int& pspd)
{
	spd=pspd;
}
void beego_control::set_rpc(int& prpc)
{
	rpc=prpc;
}
//set sensor value
void beego_control::set_acc_gyro(void)
{
	int acc_x=0, acc_y=0, acc_z=0, gyro_x=0, gyro_y=0, gyro_z=0;
	
	get_acc_gyro(hCommMotor, MOTOR_DRIVER_NUMBER, &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z);
	//set encorders
	set_acc_x(acc_x);
	set_acc_y(acc_y);
	set_acc_z(acc_z);
	set_gyro_x(gyro_x);
	set_gyro_y(gyro_y);
	set_gyro_z(gyro_z);
}
void beego_control::set_acc_x(int& pacc_x)
{
	acc_x=pacc_x;
}
void beego_control::set_acc_y(int& pacc_y)
{
	acc_y=pacc_y;
}
void beego_control::set_acc_z(int& pacc_z)
{
	acc_z=pacc_z;
}
void beego_control::set_gyro_x(int& pgyro_x)
{
	gyro_x=pgyro_x;
}
void beego_control::set_gyro_y(int& pgyro_y)
{
	gyro_y=pgyro_y;
}
void beego_control::set_gyro_z(int& pgyro_z)
{
	gyro_z=pgyro_z;
}


//publish encorder values
void beego_control::publish_encorders(void)
{
	//value of encorder(ros msgs)
	std_msgs::Int32 msg_st;
	std_msgs::Int32 msg_fet;//temperature of FET
	std_msgs::Float32 msg_bt;//Motor Driver Battery Voltage
	std_msgs::Int32 msg_vol;//Current Voltage
	std_msgs::Int32 msg_spd;//speed
	std_msgs::Int32 msg_rpc;//RPC(?)
	//set messages
	msg_st.data=st;
	msg_fet.data=fet;
	msg_bt.data=bt;
	msg_vol.data=vol;
	msg_spd.data=spd;
	msg_rpc.data=rpc;
	//value of sensor(ros msgs)
	geometry_msgs::Point32 msg_acc;
	geometry_msgs::Point32 msg_gyro;
	//set messages
	msg_acc.x=acc_x;
	msg_acc.y=acc_y;
	msg_acc.z=acc_z;
	
	msg_gyro.x=gyro_x;
	msg_gyro.y=gyro_y;
	msg_gyro.z=gyro_z;
	std::cout<<"gyro "<<atan(acc_y/std::sqrt(acc_x*acc_x+acc_z*acc_z))*180/M_PI<<" "<<atan(acc_x/std::sqrt(acc_y*acc_y+acc_z*acc_z))*180/M_PI<<"\n";
	//received data
	std::cout<<"received data:"<<rcvdata<<"\n";
	//
	std::cout<<"order_msg:"<<order_msg;//<<"\n";
	//publish messagegs
	std::cout<<"msg_st:"<<msg_st;//<<"\n";
	pub_st.publish(msg_st);
	std::cout<<"msg_fet:"<<msg_fet;//<<"\n";
	pub_fet.publish(msg_fet);
	std::cout<<"msg_bt:"<<msg_bt;//<<"\n";
	pub_bt.publish(msg_bt);
	std::cout<<"msg_vol:"<<msg_vol;//<<"\n";
	pub_vol.publish(msg_vol);
	std::cout<<"msg_spd:"<<msg_spd;//<<"\n";
	pub_spd.publish(msg_spd);
	std::cout<<"msg_rpc:"<<msg_rpc;//<<"\n";
	pub_rpc.publish(msg_rpc);
	
	std::cout<<"msg_acc:"<<msg_acc;//<<"\n";
	pub_acc.publish(msg_acc);
	std::cout<<"msg_gyro:"<<msg_gyro;//<<"\n";
	pub_gyro.publish(msg_gyro);
	
}




