// DecisionMakingx.h: interface for the CDecisionMakingx class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_DECISIONMAKINGX_H__E1F060A5_484F_4338_B877_7B533844419F__INCLUDED_)
#define AFX_DECISIONMAKINGX_H__E1F060A5_484F_4338_B877_7B533844419F__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "globe.h"
#include "defines.h"
#include "GeometryR.h"
#include "Logger.h"

#include "Formation.h"
#include "network.h"
#include "CounterTimer.h"
#include "Hundun.h"

#include "PerfTimer.h"


#define BALLNUMPARA	7

 //long x,y;
class CDecisionMakingx  : CCounterTimerListener
{
private:
	// Basic Actions
	int TurnToPointPDli(dbROBOTPOSTURE *pRobot,dbPOINT Point,int clock,dbLRWheelVelocity *pSpeed)	;
	void TurnToAnglePD(int nRobot, double dAngle, int clock);
	void TurnToPointPD(int nRobot, VecPosition posTarget, int clock);
	int ToPositionPDli(dbROBOTPOSTURE* pROBOTPOSTURE,dbPOINT Target,double same_speed, double end_speed,dbLRWheelVelocity* pLRWheelVelocity);
	int	TurnToAnglePDli(dbROBOTPOSTURE *pRobot,double dbAngle,int clock,dbLRWheelVelocity *pSpeed);
	void Turn(int nRobot, double dVelocity,int clock);
	void MoveOnAngle(int nRobot,double Angle,double speed);
	double TNparali(RobotInford myrobot,dbPOINT ball,dbPOINT goal,double dist,double angle);
	void MoveToPt(int nRobot,VecPosition posTarget,double speed);
	int Move2Ptli(dbROBOTPOSTURE* pROBOTPOSTURE,dbPOINT Target,double speed,dbLRWheelVelocity* pLRWheelVelocity);
	void ToPositionPD(int nRobot, VecPosition posTarget, double speed);

	void ToPositionN(int nRobot, VecPosition posTarget, double speed);
	void ToPositionNew(int nRobot, VecPosition posTarget, VecPosition posBall, double speed, int IfEndprocess);
	int ToPositionNewli(dbROBOTPOSTURE* robot, ballInformation ball,dbPOINT directionpt, double samespeed, int IfEndprocess,dbLRWheelVelocity* pSpeed);
	double	GetCrossPtWithGLLineli(double x)	;
	//末端处理
	void EndProcess(int i,int nRobot,VecPosition posTarget, VecPosition posBall);
	int EndProcessli(dbROBOTPOSTURE *pRobot,dbPOINT shoot_target,dbPOINT ballPt,dbLRWheelVelocity *pSpeed);

	
	void ToPositionPDGoal(int nRobot, VecPosition posTarget, double speed, double endspeed);
	int ToPositionPDGoalli(dbROBOTPOSTURE* pROBOTPOSTURE,dbPOINT Target,double startspeed,double endspeed,dbLRWheelVelocity* pLRWheelVelocity);
	void ToPositionAvoidObstacles(int nRobot, VecPosition pos[], int nObstacles);
MoveParameter m_MoveParameter;
private:
	// Support

private:
	// Complex Actions
	void Vect_MidShoot(int nRobot);
	void Vect_MidShoot1(int nRobot);
	void Vect_MidShoot2(int nRobot);
	int Vect_MidShootli(dbROBOTPOSTURE pRobotInford, BallInformation &ball,dbLRWheelVelocity *pSpeed);
	void Wait(int nRobot, VecPosition posTarget, double angle, double speed);
	void Wait(int nRobot, VecPosition posTarget, VecPosition posToward, double speed);
	
	void ShootLine(int nRobot);

	void KickBall(int nRobot);
	
	void RobotReturn2Pt();
	void RobotReturn2bound();
	void Round(int nRoundRobot);

	int		GetBoundAreaNo(VecPosition posBall);
	void	BoundPushBall(int nRobot);

	void	PushBallofBound(int nRobot);

	// Gaolie
	void GoalieAction(int nRobot);	
	int GoalieAction3(dbROBOTPOSTURE *pRobotInford, int x, BallInformation ball, dbLRWheelVelocity *pSpeed,int nRobot);
	int GoalieAction4(dbROBOTPOSTURE *pRobotInford, int x, BallInformation ball, dbLRWheelVelocity *pSpeed,int nRobot);
	double AdjustTarget();
	double GetCrossPtWithGLLine();
	int Forcastball(dbPOINT *pPoint,FORCASTBALL *pBall);
	
public:
	// Strategy Functions
	void Initialize();
	DecisionParamter m_decisionparamter;
	void InitDEG(DEGame DEG);
	void Startx(RobotInford dmRobot[]);
	double m_Front;
	AngleParameter m_AngleParameter;
	
private:
	void	PreProcess();
	void	MiroSot_DecisionMaking();
	int		GetAreaNo(VecPosition posBall);
	void	TaskDecompose(int areaNo);
	void	FormInterpret(int formNo);
	void	CharInterpret(int nRobot,int charNo, dbLRWheelVelocity* pSpeed);
	void	CharAllot();
	double	CharPerformance(int charNo, int nRobot);
	int		GetBestRobot(double Performance[], int charNo);
	void	RobotManager();
	void	ActProcess();

	

private:
	// Support
	void LogData();
	void Test();
	
	// Predict
	int KalmanFilter();
	
	VecPosition PredictBall(int nCycle, VecPosition* posVel=NULL);

private:
	// Performance from predict
	int GetToPositionNCycle(int nRobot, VecPosition posTarget);
	int GetToPositionNewCycle(int nRole, int nRobot, VecPosition posBall, VecPosition posTarget);
	
	int	GetToPositionNewPerformance(int nRole, int nRobot, 
		VecPosition posTarget, VecPosition& posBall);

	int	GetKickBallPerformance(int nRole, int nRobot, VecPosition& posBall);

	int GetShootLinePerformance(int nRobot);

	void LimitSpeed(dbLRWheelVelocity* pSpeed, double dMaxSpeed, BOOL bMax = FALSE);
	
	
	VecPosition m_posBall;
	
	CFormation	m_formation;
	int			m_nTimer, m_nTimerLastStart;
	
	network		nettpt, nettpn;
public:
	CDecisionMakingx();
	virtual ~CDecisionMakingx();

	BOOL				nReset;	//是否归位,FALSE
	BOOL				together,round;		///< 赛前跑位
	int					m_nTestRobot;
	RobotInford			Robot[2*ROBOTNUMBER+1], oldRobot[2*ROBOTNUMBER+1];
	dbLRWheelVelocity	rbV[ROBOTNUMBER], oldrbV[ROBOTNUMBER];

private:
	dbPOINT				ball;
	FORCASTBALL         ballCharacter;
	dbPOINT				oldBallPt[BALLNUMPARA],FilterBallPt[BALLNUMPARA],FilterVel[BALLNUMPARA],FilterAccel[BALLNUMPARA];
	int					sym[3];				///< 视觉丢球处理
	
	DEGame				dmDEG;

private:
	int					dmformNo;	///< 队形
	
	///< 队形排列
	int	currentForm[ROBOTNUMBER],oldForm[ROBOTNUMBER];

	///< 按照机器人顺序排列时对应的角色排列
	int	currentResult[ROBOTNUMBER], oldResult[ROBOTNUMBER];
	
	///< 按照角色顺序排列时对应的机器人排列
	int	currentOrder[ROBOTNUMBER],oldOrder[ROBOTNUMBER];

	int	m_nPriorityForm[ROBOTNUMBER], m_nPriorityResult[ROBOTNUMBER];

	double	perfRecord[ROBOTNUMBER],oldperfRecord[ROBOTNUMBER];
	int		effecChar[100];

	Logger	log;
//	CPerfTimer m_PerfTimer;

private:
	BOOL bStartUnUsual;

	BOOL m_bShootLine[ROBOTNUMBER];
	BOOL m_bRobotShoot[ROBOTNUMBER];

	VecPosition		m_posPostBall;
	BOOL			m_bBallKick;

	CCounterTimer	m_timerStart, m_timerShootLine, m_timerBallKick;
	virtual void Update(CCounterTimer* Timer);

private:
	
	void Mark(int nRobot);
	VecPosition GetMarkPosition();

	void SendMsg(int nLevel, char* msg, ... );
	
	// Hundun obstacle avoidance
private:
	CHunDun	m_hundun;
	void	HundunTest();
	void	Capture();
	

	// SVM
private:
	struct svm_model* m_svmModelTpt;
	struct svm_node*  m_svmAttrTpt;
	
private:
	double	Getpt2ptAngle(dbPOINT pt1,dbPOINT pt2);

//	Kick	m_kick[ROBOTNUMBER];
};
#endif // !defined(AFX_DECISIONMAKINGX_H__E1F060A5_484F_4338_B877_7B533844419F__INCLUDED_)





















