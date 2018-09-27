#ifndef _ROBOTSTRUCT_H_
#define _ROBOTSTRUCT_H_

typedef unsigned char byte;


typedef struct
{
	float sonar_1;
	float sonar_2;
	float sonar_3;
	float sonar_4;
}RobotSonar;

typedef struct
{
	float yaw;
	float patch;
	float roll;
}RobotIMU;

typedef struct
{
	float x;
	float y;
	float th;
	float v;
	float vth;
}RobotOdom;

typedef struct
{
	byte MsgType;//1byte
	byte RobotState;//1byte
	RobotOdom Odom;//20byte
	RobotSonar Sonar;//16byte
	RobotIMU IMU;//12byte
}RobotSubMsg;

typedef struct
{
	byte MsgType;//1byte
	byte RobotState;//1byte
	float vx;//4byte
	float vth;//4byte
}RobotPubMsg;

#endif // !_ROBOTSTRUCT_H_
