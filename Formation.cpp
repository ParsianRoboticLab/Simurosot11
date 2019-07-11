// Formation.cpp: implementation of the CFormation class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"

#include "Formation.h"

#include "Parse.h"
#include <fstream>	 // ifstream

#include "Defines.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CFormation::CFormation()
{

}

CFormation::~CFormation()
{
}

BOOL CFormation::ReadFormation(LPCTSTR lpszPathName)
{
	std::ifstream infile(lpszPathName);
	if (!infile)
		return FALSE;

	char strLine[256], *str;
	int nType = 0, nLine = 0;
	
	while (infile.getline(strLine, sizeof(strLine)))
	{
		str = &strLine[0];
		if( !(strLine[0] == '\n' || strLine[0] == '#' || strLine[0]=='\0' ||
			Parse::GotoFirstNonSpace( &str ) == '\0' ) )
		{
			switch (nType)
			{
			case 0:
				{
					m_nGameType = Parse::ParseFirstInt(&str);
					nType = 1;

					switch (m_nGameType)
					{
					case 0:
						{
							m_nRobotNumber = 3;
						}
						break;
					case 1:
					case 2:
						{
							m_nRobotNumber = 5;
						}
						break;
					case 3:
						{
							m_nRobotNumber = 11;
						}
						break;
					}
				}
				break;
			case 1:
				{
					m_nFormNumber = Parse::ParseFirstInt(&str);
					nType = 2;
				}
				break;
			case 2:
				{
					int nArea = Parse::ParseFirstInt(&str);
					for (int i=0; i<m_nRobotNumber-1; i++)
						m_nForm[nArea][i] = Parse::ParseFirstInt(&str);
					nLine++;
					if (nLine == m_nFormNumber)
						nType = 3;
				}
				break;
			case 3:
				{
					m_nRoleNumber = Parse::ParseFirstInt(&str);
					nType = 4;
						
					nLine = 0;
				}
				break;
			case 4:
				{
				/*	int nRole = Parse::ParseFirstInt(&str);
	
					m_nRolePerformance[nRole] = Parse::ParseFirstInt(&str);
					for (int i=0; i<2; i++)
						m_nRolePerformanceParameters[nRole][i] = Parse::ParseFirstDouble(&str);
					
					nLine++;
					if (nLine == m_nRoleNumber)
				*/	{
						nType = 5;
						nLine = 0;
					}
				}
				break;
			case 5:
				{
					int nRole = Parse::ParseFirstInt(&str);
					
					m_nRoleExecute[nRole] = Parse::ParseFirstInt(&str);

					for (int i=0; i<12; i++)
						m_nRoleExecuteParameters[nRole][i] = Parse::ParseFirstDouble(&str);

					nLine++;
					if (nLine == m_nRoleNumber)
					{
						nType = 6;
					}
				}
				break;
			default:
				break;
			}
		}
	}

	return TRUE;
}

BOOL CFormation::GetStrategyPosition(int nRole, VecPosition posBall,
								   VecPosition& posRobot, double& angleRobot, int& nAction2)
{
	switch(m_nRoleExecute[nRole])
	{
	case 1:
		{
			double x,y,dx,dy,ax,ay,minx,maxx,miny,maxy;
			BOOL bButtonSide = TRUE;
			
			x = m_nRoleExecuteParameters[nRole][0];
			y = m_nRoleExecuteParameters[nRole][1];
			
			// 与某角色关于y轴对称
			if (x == -1)
			{
				nRole = (int)y;
				x = m_nRoleExecuteParameters[nRole][0];
				y = m_nRoleExecuteParameters[nRole][1];
				bButtonSide = FALSE;
			}
			
			dx = m_nRoleExecuteParameters[nRole][2];
			dy = m_nRoleExecuteParameters[nRole][3];
			ax = m_nRoleExecuteParameters[nRole][4];
			ay = m_nRoleExecuteParameters[nRole][5];
			minx = m_nRoleExecuteParameters[nRole][6];
			maxx = m_nRoleExecuteParameters[nRole][7];;
			miny = m_nRoleExecuteParameters[nRole][8];
			maxy = m_nRoleExecuteParameters[nRole][9];
			
			posRobot.SetVecPosition(x+ax*(posBall.GetX()+dx), 
				y+ay*(posBall.GetY()+dy));
			
			if (maxx == 0)
				maxx = PITCH_LENGTH;
			if (maxy == 0)
				maxy = PITCH_WIDTH;
			
			if (posRobot.GetX() < minx)
				posRobot.SetX(minx);
			if (posRobot.GetX() > maxx)
				posRobot.SetX(maxx);
			if (posRobot.GetY() < miny)
				posRobot.SetY(miny);
			if (posRobot.GetY() > maxy)
				posRobot.SetY(maxy);
			
			if (m_nRoleExecuteParameters[nRole][10] == 10)
			{
				angleRobot = (posBall - posRobot).GetDirection();
			}
			else if (m_nRoleExecuteParameters[nRole][10] == 20)
			{
				// 球门
				VecPosition posGoal(PITCH_LENGTH, PITCH_WIDTH/2);
				angleRobot = (posGoal - posRobot).GetDirection();
			}
			else
			{
				angleRobot = PI * m_nRoleExecuteParameters[nRole][10] / 180.;
			}
			
			if (!bButtonSide)
			{
				posRobot.SetY(PITCH_WIDTH - posRobot.GetY());
				angleRobot = -angleRobot;
			}

			nAction2 = static_cast<int>(m_nRoleExecuteParameters[nRole][11]);
		}

		return TRUE;
		break;
	}
	
	return FALSE;
}
