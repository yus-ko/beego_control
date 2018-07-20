//
// Sample for beego(motor driver for MDR8)
// Main Program
//
// Beego : Information
// Motor Encorder 
// Motor Gear Ratio  27:1
// Gear Tooth  16 (Motor)
// Gear Tooth  45 (Tire)
// Tire Diameter 82mm
//
// Copyright (C) 2018 Mobile Robot Research Co. LTD. Allright Reserved.
//
//
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <sys/types.h>
#include <math.h>
#include <signal.h>
#include <sys/time.h>
#include <linux/serial.h>

#include "iroboserial.h"


// Defines
#define MOTOR_DRIVER_PORT				0			// Port Number
#define MOTOR_DRIVER_BAUDRATE			115200		// Baudrate

#define MOTOR_DRIVER_NUMBER				0			// Motor driver number
#define MOTOR_LEFT						0			// Motor Left
#define MOTOR_RIGHT						1			// Motor Right

#define MAX_PORT_NAME					32			// Port Name Buffer Size
#define MDR8_CMD_SIZE					128			// Motor Driver Commnad Buffer Size
#define MDR8_BUFF_SIZE					512			// Motor Driver Return Data Buffer Size
#define MDR8_TIMEOUT					500			// Motor Driver Read Time Out
#define MDR8_READ_LENGTH				4			// Motor Driver Retrun Pakect Read Length
#define MDR8_RETURN_OK					"OK"		// Motor Driver Return "OK" Code
#define MDR8_MOTOR_CMT					0			// Motor Driver MT Command Immediately
#define MDR8_MOTOR_CMB					1			// Motor Driver MT Command Stored
// CMD/CMB Mode
#define MDR8_MOTOR_MODE_CCURRENT		0			// Current Mode
#define MDR8_MOTOR_MODE_SPEED			1			// Speed Mode
#define MDR8_MOTOR_MODE_POSITION		2			// Position Mode
#define MDR8_MOTOR_MODE_TORQUE_OFF		3			// Torque Off
#define MDR8_MOTOR_MODE_TORQUE_ON		4			// Torque On

// Beego Information
#define BEEGO_MOTOR_ENCORDER			512			// Beego Motor Encorder 512
#define BEEGO_MOTOR_GEAR				26.85		// Beego Motor Gear Ration 3249:121
#define BEEGO_MOTOR_GEAR_TOOTH			16			// Beego Motor Gear Tooth 16
#define BEEGO_TIRE_TOOTH				45			// Beego Tire Tooth 45
#define BEEGO_TIRE_DIAMETER				82			// Beego Tire Diameter 82mm
#define BEEGO_TIRE_DISTANCE				290			// Beego Tire Distance 290mm


// Variables
bool gbTorque = false;								// Motor Torque Flag
bool gbTerminate = false;							// loop flag


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
int MDR8SendRead(HANDLE hComm, char *psend, int sendlen, char *prcv, int rcvlen, int timeout)
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
bool MDR8SendCommand(HANDLE hComm, char *psend, int sendlen, char *prcv, int rcvlen, int timeout)
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
bool MDR8MspSend(HANDLE hComm, int drv, int motor, int maxspeed)
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
bool MDR8SocSend(HANDLE hComm, int drv, int motor, int current, int ctime)
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
bool MDR8RacSend(HANDLE hComm, int drv, int motor)
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
bool MDR8MotorControl(HANDLE hComm, int mode, int ctrl, int drv, int motor, int val)
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
int MDR8GetStatusLong(HANDLE hComm, int drv, int motor, int *pst, int *pfet, int *pbt, int *pvol, int *pspd, int *prpc)
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

  return ret;
}

//
//	MDR8BcpSend(HANDLE hComm)
//	MDR8 BCP Send
//
//	HANDLE hComm				Serial Handle
//	return bool					status
//
bool MDR8BcpSend(HANDLE hComm)
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
//   main
//
//
int main(int argc, char *argv[])
{
	HANDLE	hCommMotor;								// Motor Driver Handle;
	char	sPort[MAX_PORT_NAME];					// Port Name
	int		iDriveSpeedRight = 0;					// Right Motor Speed
	int		iDriveSpeedLeft = 0;					// Left Motor Speed
	struct timeval	times, timec;					// for Timer
	long	lElapsedTime = 0;						// Elapsed Time
	long	lOldTime = 0;							// Old Time
	int32_t	iMoveStep = 0;							// Moving Step
	int32_t iPosition = 0;							// Right Motor Encoder Count;


	printf("\nProgram Start ....\n");
	gbTorque = false;
	gbTerminate = false;

	//
	// set port name
	//
	void setttyportname(int num)
	{
		if(num >= 0){
			sprintf(sPort, "/dev/ttyUSB%d", num);
			printf("DRIVER PORT NAME : [%s]\n", sPort);
		}
	}

	//
	// get elapsed time(miri sec)
	//
	long getMsecTime(struct timeval *pold, struct timeval *pnew)
	{
		long	mtime, seconds, useconds;

		seconds  = pnew->tv_sec  - pold->tv_sec;
		useconds = pnew->tv_usec - pold->tv_usec;
		mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;

		return mtime;
	}


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
	while(!gbTerminate){
		// STOP for CTRL+C

		// calculate elapsed time
		gettimeofday(&timec, NULL);
		lElapsedTime = getMsecTime(&times, &timec);

		if(iMoveStep == 0){
			printf("Torque ON\n");
			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_TORQUE_ON, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 0);
			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_TORQUE_ON, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 0);
			gettimeofday(&times, NULL);
//			iMoveStep = 1;
			iMoveStep = 4;
			printf("=====> Straight ... 5 sec\n");
		} else if(iMoveStep == 1){
			if(lElapsedTime < 5000){
				MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, -3000);
				MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 3000);
				MDR8BcpSend(hCommMotor);
			} else {
				gettimeofday(&times, NULL);
				iMoveStep = 2;
				printf("=====> Turning ... 3 sec\n");
			}
		} else if(iMoveStep == 2){
			if(lElapsedTime < 3000){
				MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 3000);
				MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 3000);
				MDR8BcpSend(hCommMotor);
			} else {
				gettimeofday(&times, NULL);
				iMoveStep = 3;
				printf("=====> Back ... 3 sec\n");
			}
		} else if(iMoveStep == 3){
			if(lElapsedTime < 3000){
				MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 3000);
				MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, -3000);
				MDR8BcpSend(hCommMotor);
			} else {
				gettimeofday(&times, NULL);
				iMoveStep = 4;
				printf("=====> STOP\n");
			}
		} else if(iMoveStep == 4){
			// STOP
			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 0);
			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 0);
			MDR8BcpSend(hCommMotor);

			// wait for STOP
			usleep(1000000);

			// Speed Change
			MDR8MspSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 1000);
			MDR8MspSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 1000);

			// Reset Counter
			printf("Reset Counter\n");
			MDR8RacSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT);
			MDR8RacSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT);

			iMoveStep = 5;
			printf("=====> Move 500mm\n");
		} else if(iMoveStep == 5){
			// 500mm
			float d = 500.0 / (float)((float)BEEGO_TIRE_DIAMETER * M_PI);
			float dg = d * (float)BEEGO_TIRE_TOOTH / (float)BEEGO_MOTOR_GEAR_TOOTH;
			float dm = dg * BEEGO_MOTOR_GEAR * BEEGO_MOTOR_ENCORDER * 4;

			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_POSITION, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, (int)-dm);
			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_POSITION, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, (int)dm);
			MDR8BcpSend(hCommMotor);
			
			if(fabs(iPosition + dm) < 128){
				// STOP
				MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 0);
				MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 0);
				MDR8BcpSend(hCommMotor);

				// wait for STOP
				usleep(1000000);

				// Speed Change
				MDR8MspSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 1000);
				MDR8MspSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 1000);

				// Reset Counter
				printf("Reset Counter\n");
				MDR8RacSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT);
				MDR8RacSend(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT);

				iMoveStep = 6;
				printf("=====> 1 Rotation\n");
			}
		} else if(iMoveStep == 6){
			// 1Rotation
			float d = (float)BEEGO_TIRE_DISTANCE / (float)BEEGO_TIRE_DIAMETER;
			float dg = d * (float)BEEGO_TIRE_TOOTH / (float)BEEGO_MOTOR_GEAR_TOOTH;
			float dm = dg * 3249.0/121.0 * BEEGO_MOTOR_ENCORDER * 4;

			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_POSITION, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, (int)-dm);
			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_POSITION, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, (int)-dm);
			MDR8BcpSend(hCommMotor);

			if(fabs(iPosition + dm) < 128){
				iMoveStep = 7;
				printf("=====> STOP\n");
			}
		} else if(iMoveStep == 7){
			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 0);
			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMB, MDR8_MOTOR_MODE_SPEED, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 0);
			MDR8BcpSend(hCommMotor);

			printf("Torque Off\n");
			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_TORQUE_OFF, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, 0);
			MDR8MotorControl(hCommMotor, MDR8_MOTOR_CMT, MDR8_MOTOR_MODE_TORQUE_OFF, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, 0);
			gettimeofday(&times, NULL);
			iMoveStep = 8;
		} else {
			// none
		}

		// Read Encorder Data
		{
			int st = 0, fet = 0, bt = 0, vol = 0, spd = 0, rpc = 0;
				
			MDR8GetStatusLong(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_LEFT, &st, &fet, &bt, &vol, &spd, &rpc);
			printf("%.1fV L:%d ", (float)bt/10.0, rpc);
			MDR8GetStatusLong(hCommMotor, MOTOR_DRIVER_NUMBER, MOTOR_RIGHT, &st, &fet, &bt, &vol, &spd, &rpc);
			printf("R:%d\n", rpc);
			iPosition = rpc;
		}

		usleep(100000);
	}

	SerialClose(hCommMotor);

	return 0;
}

