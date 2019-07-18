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

class CHunDun {
// Construction
public:
	CHunDun(double x = 0, double y = 0, double x_goal = 0, double y_goal = 0);

// Attributes
public:

// Operations
public:
	UINT MapRand(UINT nMax);
	
	double yg;        // Ŀ���λ��
	double xg;
	
	int iz;    // �ϰ������
	
	double XYT1[2];    // ��һ���ڻ���������
	double XYTO[2];    // ��ǰ����������
	
	double XYOX[20];    // �ϰ���X����
	double XYOY[20];    // �ϰ���Y����
	
	double X[N];        // ���벽��[0]�ͽǶ�[1]
	
	double OPTX[N];        // ��ʱ����
	double COEA[N];        // ��ʱ����
	double XX[N][M];    // ��ʱ����
	double F[M];        // ��ʱ����
	double B[N];        // ��ʱ����
	double A[N];        // ��ʱ����
	double MINU;        // ��ʱ����
	
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
