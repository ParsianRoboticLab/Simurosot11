#ifndef _GLOBE_H_
#define _GLOBE_H_

#include "math.h"

#define PI 3.14159265358979323846
#define sqrt2 1.414213562373

#define CLOCKWISE 1
#define ANTICLOCK -1
#define NOCLOCK  0

typedef struct
{
	double x;
	double y;
}dbPOINT,BallInformation,ballInformation;

//场地物体浮点位姿信息
typedef struct
{
	double x;	//物体中心的坐标x
	double y;  //物体中心的坐标y,该坐标是以显示坐标为基准的
	double theta;//弧度方向角
	double speedv;	// 机器人线速度（上周期）
	double speedw;	// 机器人角速度
}dbROBOTPOSTURE, RobotInford;

//小车轮速值浮点型
typedef struct
{
	double LeftValue;
	double RightValue;
}dbLRWheelVelocity;

typedef struct
{
	double a;
	double b;
	double c;
}Formulation, LINEFORMULATION;

typedef struct
{
	Formulation Formu;
	double angle;
	double velocity;
	dbPOINT proBall;
} FORCASTBALL;

struct DEGame
{
	int	DEGameGround;
	int	DEStartState;
	int	DEStartMode;
	int DEPenaltyDirection;
};

#endif	// _GLOBE_H_
