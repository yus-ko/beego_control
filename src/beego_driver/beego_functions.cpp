#include<beego_control.h>

//using namespace beego;

// Functions

//	MDR8SendRead(HANDLE hComm, char *psend, int sendlen, char *prcv, int rcvlen, int timeout)
//	Send And Read
//
//	HANDLE hComm				Serial Handle
//	char *psend					Send Data
//	int sendlen					Send Data Length
//	char *prcv					Recieve Data
//	int rcvlen					Recieve Data Length
//	int timeiout				Recieve Data Timeout
//	return bool					status
//

int beego::control::MDR8SendRead(HANDLE hComm, char *psend, int sendlen, char *prcv, int rcvlen, int timeout)
{
	int ret = 0;

	if(SerialSend(hComm, psend, sendlen) != sendlen){
		printf("MDR8Send Error #send\n");
		return -1;
	}

	if((ret = SerialRead(hComm, prcv, rcvlen, timeout)) <= 0){
		printf("MDR8SendRead Error #read\n");
		return -1;
	}

	return ret;
}

//	MDR8SendCommand(HANDLE hComm, char *psend, int sendlen, char *prcv, int rcvlen, int timeout)
//	Serial Data Send & Wait
//
//	HANDLE hComm				Serial Handle
//	char *psend					Send Data
//	int sendlen					Send Data Length
//	char *prcv					Recieve Data
//	int rcvlen					Recieve Data Length
//	int timeiout				Recieve Data Timeout
//	return bool					status
//
bool beego::control::MDR8SendCommand(HANDLE hComm, char *psend, int sendlen, char *prcv, int rcvlen, int timeout)
{
	int ret = 0;

	ret = MDR8SendRead(hComm, psend, sendlen, prcv, rcvlen, timeout);
	if(ret <= 0){
		return false;
	}
	if(strncmp(prcv, MDR8_RETURN_OK, strlen(MDR8_RETURN_OK))){
		printf("MDR8SendRead Error! not OK\n");
		return false;
	}

	return true;
}

//
//	MDR8MspSend(HANDLE hComm, int drv, int motor, int maxspeed)
//	MDR8 MSP Data Send
//	Set MAX Speed
//
//	HANDLE hComm				Serial Handle
//	int drv						Motor Driver Number
//	int motor					Motor Number
//	int maxspeed				Max Motor Speed
//	return bool					status
//
bool beego::control::MDR8MspSend(HANDLE hComm, int drv, int motor, int maxspeed)
{
	char	cmdbuf[MDR8_CMD_SIZE];
	char	rcvbuf[MDR8_BUFF_SIZE];
	bool	ret = false;

	sprintf(cmdbuf, "MSP %d %d %d\r\n", drv, motor, maxspeed);
	memset(rcvbuf, 0x00, sizeof(rcvbuf));
	ret = MDR8SendCommand(hComm, cmdbuf, strlen(cmdbuf), rcvbuf, MDR8_READ_LENGTH, MDR8_TIMEOUT);

	return ret;
}

//
//	MDR8SocSend(HANDLE hComm, int drv, int motor, int current, int time)
//	MDR8 SOC Data Send
//	Set Over Current
//
//	HANDLE hComm				Serial Handle
//	int drv						Motor Driver Number
//	int motor					Motor Number
//	int current					Max Motor Current
//	int ctime					Max Motor Current Time
//	return bool					status
//
bool beego::control::MDR8SocSend(HANDLE hComm, int drv, int motor, int current, int ctime)
{
	char	cmdbuf[MDR8_CMD_SIZE];
	char	rcvbuf[MDR8_BUFF_SIZE];
	bool	ret = false;

	sprintf(cmdbuf, "SOC %d %d %d %d\r\n", drv, motor, current, ctime);
	memset(rcvbuf, 0x00, sizeof(rcvbuf));
	ret = MDR8SendCommand(hComm, cmdbuf, strlen(cmdbuf), rcvbuf, MDR8_READ_LENGTH, MDR8_TIMEOUT);

	return ret;
}

//
//	MDR8RacSend(HANDLE hComm, int drv, int motor)
//	MDR8 RAC Data Send
//	Reset Angle Counter
//
//	HANDLE hComm				Serial Handle
//	int drv						Motor Driver Number
//	int motor					Motor Number
//	return bool					status
//
bool beego::control::MDR8RacSend(HANDLE hComm, int drv, int motor)
{
	char	cmdbuf[MDR8_CMD_SIZE];
	char	rcvbuf[MDR8_BUFF_SIZE];
	bool	ret = false;

	sprintf(cmdbuf, "RAC %d %d\r\n", drv, motor);
	memset(rcvbuf, 0x00, sizeof(rcvbuf));
	ret = MDR8SendCommand(hComm, cmdbuf, strlen(cmdbuf), rcvbuf, MDR8_READ_LENGTH, MDR8_TIMEOUT);

	return ret;
}

//
//	MDR8MotorSpeed(HANDLE hComm, int mode, int ctrl, int drv, int motor, int speed)
//	MDR8 CMB/CMT Data Send
//	Motor Speed Control
//
//	HANDLE hComm				Serial Handle
//	int mode					Motor CMB/CMT Command Mode
//	int ctrl					Motor Driver Control Mode
//	int drv						Motor Driver Number
//	int motor					Motor Number
//	int val						Motor Speed or Position .. Other 0
//	return bool					status
//
bool beego::control::MDR8MotorControl(HANDLE hComm, int mode, int ctrl, int drv, int motor, int val)
{
	char	cmdbuf[MDR8_CMD_SIZE];
	char	rcvbuf[MDR8_BUFF_SIZE];
	bool	ret = false;

	if(mode == MDR8_MOTOR_CMB){
		sprintf(cmdbuf, "CMB %d %d %d %d\r\n", drv, motor, ctrl, val);
	} else {
		sprintf(cmdbuf, "CMT %d %d %d %d\r\n", drv, motor, ctrl, val);
	}
	memset(rcvbuf, 0x00, sizeof(rcvbuf));
	ret = MDR8SendCommand(hComm, cmdbuf, strlen(cmdbuf), rcvbuf, MDR8_READ_LENGTH, MDR8_TIMEOUT);

	return ret;
}


//
//	MDR8GetStatusLong(HANDLE hComm, int drv, int motor, int *pst, int *pfet, int *pbt, int *pvol, int *pspd, int *prpc)
//	MDR8 DSR Data Send & Recieve
//
//	HANDLE hComm				Serial Handle
//	int drv						Motor Driver Number
//	int motor					Motor Number
//	int *pst					Motor Control Mode
//	int *pfet					FET Tempture
//	int *pbt					Motor Driver Battery Voltage
//	int *pvol					Current Voltage
//	int *pspd					Speed
//	int *prpc					RPC
//	return bool					status
//
int beego::control::MDR8GetStatusLong(HANDLE hComm, int drv, int motor, int *pst, int *pfet, int *pbt, int *pvol, int *pspd, int *prpc)
{
  char	cmdbuf[MDR8_CMD_SIZE];
  char	rcvbuf[MDR8_BUFF_SIZE];
  bool	ret = false;
  int	ret_drv, ret_mt, ret_tm, ret_cur;
  
  sprintf(cmdbuf, "DSR %d %d\r\n", drv, motor);
  memset(rcvbuf, 0x00, sizeof(rcvbuf));
  ret = MDR8SendRead(hComm, cmdbuf, strlen(cmdbuf), rcvbuf, sizeof(rcvbuf), MDR8_TIMEOUT);
  if(ret > 0){
	sscanf(rcvbuf, "DS %d %d %d %d %d %d %d %d %d %d", &ret_drv, &ret_mt, pst, pfet, pbt,  &ret_cur, pvol, pspd, prpc, &ret_tm);
  }
  //aa=ret_tm;
  sample_time_c=ret_tm;
  ros_sample_time = ros::Time::now();
  return ret;
}

//
//	MDR8BcpSend(HANDLE hComm)
//	MDR8 BCP Send
//
//	HANDLE hComm				Serial Handle
//	return bool					status
//
bool beego::control::MDR8BcpSend(HANDLE hComm)
{
  char	cmdbuf[MDR8_CMD_SIZE];
  char	rcvbuf[MDR8_BUFF_SIZE];
  bool	ret = false;
  
  sprintf(cmdbuf, "BCP\r\n");
  memset(rcvbuf, 0x00, sizeof(rcvbuf));
  ret = MDR8SendCommand(hComm, cmdbuf, strlen(cmdbuf), rcvbuf, MDR8_READ_LENGTH, MDR8_TIMEOUT);

  return ret;
}

//

//
// set port name
//
void beego::control::setttyportname(int num)
{
	if(num >= 0){
		sprintf(sPort, "/dev/ttyUSB%d", num);
		printf("DRIVER PORT NAME : [%s]\n", sPort);
	}
}

//
// get elapsed time(miri sec)
//
long beego::control::getMsecTime(struct timeval *pold, struct timeval *pnew)
{
	long	mtime, seconds, useconds;

	seconds  = pnew->tv_sec  - pold->tv_sec;
	useconds = pnew->tv_usec - pold->tv_usec;
	mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;

	return mtime;
}

//get acc jyro data
int beego::control::get_acc_gyro(HANDLE hComm, int drv, int *acc_x, int *acc_y, int *acc_z, int *gyro_x, int *gyro_y, int *gyro_z)
{
  char	cmdbuf[MDR8_CMD_SIZE];
  char	rcvbuf[MDR8_BUFF_SIZE];
  bool	ret = false;
  int	ret_drv, ret_tm, ret_cur;
  
  sprintf(cmdbuf, "MSR %d\r\n", drv);
  memset(rcvbuf, 0x00, sizeof(rcvbuf));
  ret = MDR8SendRead(hComm, cmdbuf, strlen(cmdbuf), rcvbuf, sizeof(rcvbuf), MDR8_TIMEOUT);
  rcvdata=std::string(rcvbuf);
  if(ret > 0){
	sscanf(rcvbuf, "MS %d %d %d %d %d %d %d %d", &ret_drv, acc_x, acc_y, acc_z, gyro_x,  gyro_y, gyro_z, &ret_tm);
	//sscanf(rcvbuf, "MSS %d %d %d %d %d %d %d", &ret_drv, acc_x, acc_y, acc_z, gyro_x,  gyro_y, gyro_z);
  }

  return ret;
}

