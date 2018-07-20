#include<headers.h>
#include<setting_parameters.h>
#include<control_functions.h>

int main(int argc,char **argv){
	ros::init(argc,argv,"test_node");
	HANDLE	hCommMotor;								// Motor Driver Handle;
	//char	sPort[MAX_PORT_NAME];					// Port Name
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
	/*
	extern"C"{
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
	}
	*/
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
