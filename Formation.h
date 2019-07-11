// Formation.h: interface for the CFormation class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_FORMATION_H__FE99B815_E1F2_45D2_ABBD_3C437F699689__INCLUDED_)
#define AFX_FORMATION_H__FE99B815_E1F2_45D2_ABBD_3C437F699689__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "GeometryR.h"

class CFormation  
{
public:
	BOOL ReadFormation(LPCTSTR lpszPathName);
	CFormation();
	virtual ~CFormation();

public:
	BOOL GetStrategyPosition(int nRole, VecPosition posBall,
					VecPosition& posRobot, double& angleRobot, int& nAction2);

	int		m_nGameType;
	int		m_nRobotNumber;
	
	int		m_nFormNumber;
	int		m_nForm[200][11];				///< nForm[nArea][nRobot]
	
	int		m_nRoleNumber;
	
	int		m_nRolePerformance[200];			///< nRolePerformance[nRole]
	double	m_nRolePerformanceParameters[200][2];	///< nRolePerformance[nRole][nParametersN]	// Max Parameter 2

	int		m_nRoleExecute[200];				///< nRoleExecute[nRole];
	double	m_nRoleExecuteParameters[200][12];		///< nRoleExecuteParameters[nRole][nParametersN]	// Max Parameter 8
};

#endif // !defined(AFX_FORMATION_H__FE99B815_E1F2_45D2_ABBD_3C437F699689__INCLUDED_)
