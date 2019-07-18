// Kick.cpp: implementation of the Kick class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Kick.h"

#include "Defines.h"

#define LOG_BEZEIR
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Kick::Kick()
{
	// 周期变长，1,2增加，0不变. T:40->60,k[1]:25->45,k[2]:5->10.
	m_dCurvature = 0.5;		// 曲率与角度调整有关
	m_dKn[0]=.010;
	m_dKn[1]=25;
	m_dKn[2]=10;
	m_dKn[3]=25;
	m_bPathValid = false;
	m_nDirection = 0;
}

void Kick::Init(RobotInford robot, VecPosition posTarget, VecPosition posBall) 
{ 
	m_bPathValid = false;
	m_nKickTime = 2;
	m_robot = robot; 
	m_posTarget = posTarget;
	m_posBall = posBall;
}


void Kick::GeneratePath(int nCycle)
{
	m_nCycle = nCycle;
	m_bPathValid = true;

	// initalization for 4 point in bezeir curve
	double vel[2]={10, 120};
	
	// 判断踢球方向，前进or倒退
	VecPosition posRobot(m_robot.x, m_robot.y);
	double angle = (m_posBall-posRobot).GetDirection()-m_robot.theta;
	if (angle<-PI/2 || angle>PI/2)	//倒向踢球
		m_nDirection = -1;

	// 计算曲率
	angle = (m_posTarget-m_posBall).GetDirection()-(m_posBall-posRobot).GetDirection();
	//m_dCurvature = -angle*2/PI;
	

	double alpha[2]={VecPosition::NormalizeAngle(m_robot.theta-PI*m_nDirection), 
					VecPosition::NormalizeAngle((m_posTarget-m_posBall).GetDirection())};		// direction

	VecPosition posInit[4];
	posInit[0] = posRobot + VecPosition(ROBOT_LENGTH, alpha[0], POLAR);
	posInit[3] = m_posBall - VecPosition((ROBOT_LENGTH+BALL_SIZE)/2, alpha[1], POLAR);
	m_dCurvature = 2./3.*(fabs(posInit[3].GetX()-posInit[0].GetX())
		+fabs(posInit[3].GetY()-posInit[0].GetY()))/(vel[0]*(sin(alpha[0])+cos(alpha[0]))
		+vel[1]*(sin(alpha[1])+cos(alpha[1])));
	posInit[1] = posInit[0] + VecPosition(cos(alpha[0]), sin(alpha[0]))*fabs(m_dCurvature)*vel[0];
	posInit[2] = posInit[3] - VecPosition(cos(alpha[1]), sin(alpha[1]))*fabs(m_dCurvature)*vel[1];

	// 计算周期
	// 首先计算曲线长度
	double dLength=0;
	double u;
	for (int i=0; i<=m_nCycle; i++)
	{
		u = double(i)/m_nCycle;
		u = 0.5*sin(u*PI-PI/2)+0.5;
		m_posPath[i] = posInit[0]*(1-u)*(1-u)*(1-u) + posInit[1]*3*(1-u)*(1-u)*u + posInit[2]*3*(1-u)*u*u + posInit[3]*u*u*u;
		if (i>0) dLength += (m_posPath[i]-m_posPath[i-1]).GetMagnitude();
	}

	// t=s/v
	m_nCycle = int(dLength/1.90);
	
	// 调整参数
	m_dKn[0] = 0.010;
	m_dKn[1] = m_nCycle-15.00;
	m_dKn[2] = m_nCycle/4-5.00;
	m_dKn[3] = m_dKn[1];
	if (m_nCycle<20)
	{
		m_nCycle = 20;
		m_dKn[0] = 0.005;
		m_dKn[1] = 15;
		m_dKn[2] = 3;
		m_dKn[3] = m_dKn[1];
	}
	
	// set path point
	for (i=0; i<=m_nCycle; i++)
	{
		u = double(i)/m_nCycle;
		u = 0.5*sin(u*PI-PI/2)+0.5;
	//	u = exp(log(2)*u)-1;
		m_posPath[i] = posInit[0]*(1-u)*(1-u)*(1-u) + posInit[1]*3*(1-u)*(1-u)*u + posInit[2]*3*(1-u)*u*u + posInit[3]*u*u*u;
		if (i==0) m_anglePath[0] = alpha[0];
		else	m_anglePath[i] = (m_posPath[i]-m_posPath[i-1]).GetDirection();
	}

	// set Vr & Wr in every point
	double dSimulationStep = 0.038;
	for (i=0; i<m_nCycle; i++)
	{
		m_dVr[i] = (m_posPath[i+1]-m_posPath[i]).GetMagnitude();
		m_dVr[i] = m_dVr[i]/dSimulationStep;

		m_dWr[i] = (m_anglePath[i+1]-m_anglePath[i]);
		m_dWr[i] = m_dWr[i]/dSimulationStep;
	}
}

void Kick::GetNextCommand(dbLRWheelVelocity* pSpeed)
{
	if (!m_bPathValid) return;

	VecPosition d;
	double v, w;

	double theta = VecPosition::NormalizeAngle(m_robot.theta-PI*m_nDirection);
	VecPosition posRobot = VecPosition(m_robot.x, m_robot.y) + VecPosition(ROBOT_LENGTH, theta, POLAR);;

	m_nKickTime++;
	if (m_nKickTime > m_nCycle+20)
	{
		pSpeed->LeftValue = 0;
		pSpeed->RightValue = 0;
	//	m_bPathValid = false;
	}
	else if (m_nKickTime > m_nCycle)
	{
		pSpeed->LeftValue = MAXSPEED;
		pSpeed->RightValue = MAXSPEED;
	}
	else
	{
		// calculate error
		d = m_posPath[m_nKickTime] - posRobot;
		e[0][0] = d.GetX() * cos(theta) + d.GetY() * sin(theta);
		e[0][1] = d.GetX() * (-sin(theta)) + d.GetY() * cos(theta);
		e[0][2] = m_anglePath[m_nKickTime] - theta;

				
		// deside robot velocity
		v = m_dVr[m_nKickTime] * cos(e[0][2]) + m_dKn[1] * e[0][0] + m_dKn[3] * e[0][1];
		w = m_dWr[m_nKickTime] + m_dKn[0] * e[0][1] * m_dVr[m_nKickTime] * sin(e[0][2])/e[0][2] + m_dKn[2] * e[0][2];

		pSpeed->LeftValue = v + w*ROBOT_LENGTH;
		pSpeed->RightValue = v - w*ROBOT_LENGTH;
	}

	if (m_nDirection==-1)
	{
		pSpeed->LeftValue = -pSpeed->LeftValue;
		pSpeed->RightValue = -pSpeed->RightValue;
	}
	
	return;
}