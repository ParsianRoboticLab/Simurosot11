// Kick.h: interface for the Kick class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_KICK_H__5BF0F724_7DD1_41A0_9E0C_8875BF0278B9__INCLUDED_)
#define AFX_KICK_H__5BF0F724_7DD1_41A0_9E0C_8875BF0278B9__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "GeometryR.h"

#define PATHPOINT 2000

class Kick  
{
public:
	Kick();
	
	void Init(RobotInford robot, VecPosition posTarget, VecPosition posBall);
	void SetPathValid(bool bPathValid=true) { m_bPathValid = bPathValid; };
	bool GetPathValid() { return m_bPathValid; };
	void SetTarget(VecPosition posTarget) { m_posTarget = posTarget; };
	VecPosition GetTarget() { return m_posTarget; };

	void SetCurvature(double dCurvature) { m_dCurvature = dCurvature; };
	void SetCycle(int nCycle=40) { m_nCycle = nCycle; };
	int	 GetCycle() { return m_nCycle; };

	void SetKickParameters(double k1, double k2, double k3) {m_dKn[0]=k1; m_dKn[1]=k2; m_dKn[2]=k3; }; 

	void GeneratePath(int nCycle = 40);
	void GetNextCommand(dbLRWheelVelocity* pSpeed);
private:
	bool	m_bPathValid;
	VecPosition m_posTarget;
	VecPosition m_posBall;
	RobotInford	m_robot;
	double	m_dCurvature;
	int		m_nCycle;
	int		m_nKickTime;
	int		m_nDirection;

	double	m_dKn[4];

	VecPosition m_posPath[PATHPOINT];
	double	m_anglePath[PATHPOINT];
	double	m_u[PATHPOINT];

	double	m_dVr[PATHPOINT];
	double	m_dWr[PATHPOINT];
	double	e[2][3];
};

#endif // !defined(AFX_KICK_H__5BF0F724_7DD1_41A0_9E0C_8875BF0278B9__INCLUDED_)
