
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
//
char	sPort[MAX_PORT_NAME];					// Port Name
	

