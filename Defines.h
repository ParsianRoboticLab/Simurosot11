#ifndef _DEFINE_H_
#define _DEFINE_H_

// Game Type
//#define SimuroSot5
#define SimuroSot11
//#define MiroSot3
//#define MiroSot5
//#define Mirosot11
typedef struct DecisionParamter1
{
	int					numPara;	
	int					StartSW;
	char				Command[3][3];
	BOOL				nReset;
	/*BallInformation		ball;
	BallInformation		oldball;
	BallInformation		oldBallPt[7];
	RobotInford			Robot[7];
	FORCASTBALL			pBall;
	*/
}DecisionParamter;

typedef struct _MoveParameter
{
	double max_distance;
	double max_distanceG;
	double V_MAX;
	double V_max;
	double max_angle;
	double kp4pospd;
	double kd4pospd;
	double kp4pospdG;
	double kd4pospdG;
}MoveParameter,MOVEPARAMETER;

typedef struct _AngleParameter
{
	double Kp;
	double Kd;
	double AngleError;//视觉所能分辨的角度，现为5度
	double MaxAngleSpeed;//小车的最大旋转边缘线速度
	double MaxAngle;//小车可以以最大角速度旋转的角度下限
	double MaxMoveSpeed;
}AngleParameter;


//薛方正
#define	STARTTIME	5
#define	NUMBER2		0 //4


///// For GoalAction
#define		G_OFFSET			1
#define     gVMAX               125.0


////// DEGameGround
#define	LeftArea	0
#define	RightArea	1

////// DEStartState
#define	Defense		1
#define	Attack		0

////// DEStartMode

#define NormalStart 0
#define PenaltyKick 1
#define GoalKick 4
#define FreeKick 2
#define FreeBall 3
#define PenaltyOnly 5

////////////////////////////////////////
//Discrib the special point such as: penalty kick ect.
#define		RPKFK_X				112.5			
#define		RPKFK_Y				65			
#define		LUP_FB_X			37.5			
#define		LUP_FB_Y1			25		


//***************************************************************//

// 和各个比赛类型相关的参数
#ifdef SimuroSot5
#define ROBOTNUMBER		5

#define PITCH_LENGTH 			220
#define PITCH_WIDTH 	 		180
#define PENALTY_AREA_LENGTH		35
#define PENALTY_AREA_WIDTH		80
#define GOAL_AREA_LENGTH		15
#define GOAL_AREA_WIDTH			50
#define GOAL_WIDTH				40
#define GOAL_DEPTH				15
#define		GATE_UP_LINE		137.29//479 
#define		GATE_DN_LINE		42.71//149
#define		GOAL_UP_LINE		116.65//407 
#define		GOAL_DN_LINE		63.34//221


#define ROBOT_LENGTH	4.5
#define BALL_SIZE		2.0


#define MAXSPEED	125

// For Areas
#define LINE1 20
#define LINE2 25
#define LINE3 60

// For Actions 
#define BUFFERDIST	5

#else 
#ifdef SimuroSot11
// 和各个比赛类型相关的参数
#define ROBOTNUMBER		11
#define		goalwidth			40	//球门长度
#define		goal_y_widthM		53.31
#define		goal_y_widthL		94.58
#define		goal_x_widthM		10.51
#define		goal_x_widthL		20
#define		GOALS_UP_LINE		116.65 
#define		GOALS_DN_LINE		63.34
#define		GOALL_UP_LINE		137.29 
#define		GOALL_DN_LINE		42.71
#define		goal_y_widthS		53.31
#define		GATE_UP_LINE		479//479 
#define		GATE_DN_LINE		149//149
#define		GOAL_UP_LINE		407//407 
#define		GOAL_DN_LINE		221//221

#define		goal_y_width		186//186
#define		goal_x_width		42//42

#define PITCH_LENGTH 			900  //场地的长
#define PITCH_WIDTH 	 		628
#define PENALTY_AREA_LENGTH		92	//大禁区	
#define PENALTY_AREA_WIDTH		390
#define GOAL_AREA_LENGTH		38	//小禁区
#define GOAL_AREA_WIDTH			330
#define GOAL_WIDTH				100	//球门
#define GOAL_DEPTH				40
#define		CENTER_X			450//450		
#define		CENTER_Y			314//314	
#define GOALWIDTH		60
//#define MAXSPEED	125
#define MAXSPEED	125

#define ROBOT_LENGTH	6
#define BALL_SIZE		3.0

// For Areas
#define LINE1 80
#define LINE2 70
#define LINE3 220
#endif

#endif



#endif	// _DEFINE_H_
//obot1 realball;

