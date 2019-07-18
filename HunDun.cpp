// HunDun.cpp : implementation file
//

#include "stdafx.h"
#include "HunDun.h"
#include "math.h"
#include <stdio.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define SeedRand() srand((UINT)::GetTickCount())
/////////////////////////////////////////////////////////////////////////////
// CHunDun

CHunDun::CHunDun(double x, double y, double x_goal, double y_goal) {
	SeedRand();
	
	A[0] = 0;
	A[1] = 0;
	
	B[0] = 20;        // ����
	B[1] = 3.1415926;    // �Ƕ�
	
	COEA[0] = 0.1;
	COEA[1] = 0.1;
	
	X[1] = 0;
}

CHunDun::~CHunDun() {
}

/////////////////////////////////////////////////////////////////////////////
UINT CHunDun::MapRand(UINT nMax) {
	int nRand = rand();
	float fMap = (float) nMax / RAND_MAX;
	float fRetVal = (float) nRand * fMap + 0.5F;
	return (UINT) fRetVal;
}

void CHunDun::CARRIERWAVE() {
	double CX[N], CY[N][M], SCY[N][M], temptminv, out;
	int R, G, i, j, flag, flag_1, il;
	flag_1 = 0;
	for (il = 0; il < 100; il++) {
		if (flag_1) {
			break;
		}
		temptminv = MINU;
		for (i = 0; i < N; i++) {
			out = ((double) MapRand(1000)) / 1000.0;
			CX[i] = out;
			for (j = 0; j < M; j++) {
				CY[i][j] = 4 * CX[i] * (1 - CX[i]);
				CX[i] = CY[i][j];
			}
		}
		for (i = 0; i < N; i++) {
			for (j = 0; j < M; j++) {
				SCY[i][j] = -1 + 2 * CY[i][j];
			}
		}
		
		for (j = 0; j < M; j++) {
			for (i = 0; i < N; i++) {
				out = ((double) MapRand(1000) / 1000.0);
				X[i] = OPTX[i] + COEA[i] * SCY[i][j] * out;
			}
			R = 1;
			G = 0;
			flag = 0;
			do {
				flag = 0;
				if ((A[R - 1] <= X[R - 1]) && (B[R - 1] >= X[R - 1])) {
					R = R + 1;
					if (R <= N)
						flag = 1;
				} else {
					G = 1;
				}
			} while (flag);
			if (G == 0)
				F[j] = fj();
			else
				F[j] = 1.0e+10;
			if (MINU >= F[j]) {
				for (i = 0; i < N; i++) {
					OPTX[i] = X[i];
				}
				MINU = F[j];
			}
		}
	}
}


double CHunDun::fj() {
	double e0 = 0.1;
	double belta = 10;
	double urep = 0;
	
	double FJ, ROURG, K, ROUO;
	double rour[10], lamda[10], ua;
	ua = 0;
	ROUO = 15;
	K = 1;
	for (int i = 0; i < iz; i++)
		lamda[i] = 50000000;
	
	XYT1[0] = XYTO[0] + X[0] * cos(X[1]);
	XYT1[1] = XYTO[1] + X[0] * sin(X[1]);
	
	for (i = 0; i < iz; i++)
		rour[i] = sqrt((XYT1[0] - XYOX[i]) * (XYT1[0] - XYOX[i]) + (XYT1[1] - XYOY[i]) * (XYT1[1] - XYOY[i]));
	
	ROURG = sqrt((XYT1[0] - xg) * (XYT1[0] - xg) + (XYT1[1] - yg) * (XYT1[1] - yg));
	ua = K * ROURG * ROURG;
	
	for (i = 0; i < iz; i++) {
		if (rour[i] < ROUO) {
			urep += lamda[i] * ((1 / rour[i] - 1 / ROUO) * (1 / rour[i] - 1 / ROUO));
		}
	}
	FJ = ua + urep;
	if (FJ < e0) {
		FJ += belta * fabs((ua - urep) / urep);
	}
	
	return FJ;
}


void CHunDun::CHAOS() {
	double CX[N], CY[N][M];
	double out = 0, TRV = 0;
	int i = 0, j = 0, k = 0, l = 0;
	for (i = 0; i < N; i++) {
		out = ((double) MapRand(1000)) / 1000.0;
		CX[i] = out;
		for (j = 0; j < M; j++) {
			CY[i][j] = 4 * CX[i] * (1 - CX[i]);
			CX[i] = CY[i][j];
		}
	}
	for (j = 0; j < M; j++) {
		for (i = 0; i < N; i++) {
			XX[i][j] = A[i] + (B[i] - A[i]) * CY[i][j];
			X[i] = XX[i][j];
		}
		F[j] = fj();
	}
	for (j = 0; j < M; j++) {
		k = j;
		for (l = j + 1; l < M; l++) {
			if (F[k] > F[l])
				k = l;
		}
		if (k != j) {
			TRV = F[k];
			F[k] = F[j];
			F[j] = TRV;
			for (i = 0; i < N; i++) {
				TRV = XX[i][k];
				XX[i][k] = XX[i][j];
				XX[i][j] = TRV;
			}
		}
	}
	for (i = 0; i < N; i++) {
		OPTX[i] = XX[i][0];
	}
	MINU = F[0];
	CARRIERWAVE();
}


void CHunDun::hundun(VecPosition pos[], int nObstacles) {
	xg = pos[1].GetX();
	yg = pos[1].GetY();
	
	XYTO[0] = pos[0].GetX();
	XYTO[1] = pos[0].GetY();
	
	for (int i = 0; i < nObstacles; i++) {
		XYOX[i] = pos[i + 2].GetX();
		XYOY[i] = pos[i + 2].GetY();
	}
	
	iz = nObstacles;
	
	CHAOS();
}
