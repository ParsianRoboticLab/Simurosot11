#if !defined(AFX_HUNDUN_H__769A125C_1ECC_11D8_901E_B7C5E14A531B__INCLUDED_)
#define AFX_HUNDUN_H__769A125C_1ECC_11D8_901E_B7C5E14A531B__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// HunDun.h : header file
//

#include "GeometryR.h"

#define N 2
#define M 50
/////////////////////////////////////////////////////////////////////////////

class CHunDun
{
// Construction
public:
	CHunDun(double x=0,double y=0,double x_goal=0,double y_goal=0);

// Attributes
public:

// Operations
public:
	UINT MapRand(UINT nMax);

	double yg;		// 目标点位置
	double xg;

	int iz;	// 障碍物个数

	double XYT1[2];	// 下一周期机器人坐标
	double XYTO[2];	// 当前机器人坐标

	double XYOX[20];	// 障碍物X坐标
	double XYOY[20];	// 障碍物Y坐标

	double X[N];		// 理想步长[0]和角度[1]

	double OPTX[N];		// 临时变量
	double COEA[N];		// 临时变量
	double XX[N][M];	// 临时变量
	double F[M];		// 临时变量
	double B[N];		// 临时变量
	double A[N];		// 临时变量
	double MINU;		// 临时变量

	void CARRIERWAVE();
	double fj();
	void CHAOS();

public:
	void hundun(VecPosition pos[], int nObstacles);
	virtual ~CHunDun();
};

/////////////////////////////////////////////////////////////////////////////

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_HUNDUN_H__769A125C_1ECC_11D8_901E_B7C5E14A531B__INCLUDED_)
