// Matrix.cpp: implementation of the Matrix class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "Matrix.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

int Matrix::MtxMultiply(double *pMtxA, double *pMtxB, int m, int n, int q, double *pMtxC) {
	double MtxA[3][3];
	double MtxB[3][3];
	double MtxC[3][3];
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			MtxA[i][j] = *pMtxA++;
		}
	}
	for (i = 0; i < n; i++) {
		for (int j = 0; j < q; j++) {
			MtxB[i][j] = *pMtxB++;
		}
	}
	for (i = 0; i < m; i++) {
		for (int j = 0; j < q; j++) {
			MtxC[i][j] = 0;
			for (int k = 0; k < n; k++)
				MtxC[i][j] += MtxA[i][k] * MtxB[k][j];
		}
	}
	for (i = 0; i < m; i++) {
		for (int j = 0; j < q; j++) {
			*pMtxC++ = MtxC[i][j];
		}
	}
	//  pMtxC=MtxC[0];
	/*	int p;
	double ti;
	double si;
	div_t  div_result;
    div_result=div(n,2);
	p=div_result.quot;
    for(int i=0;i<m;i++)
	{
	for(int j=0;j<p;j++)
	{
	ti=MtxA[i][2*j]*MtxA[i][2*j+1];
	}
	}
	for(i=0;i<q;i++)
	{
	for(int j=0;j<p;j++)
	{
	si=MtxB[2*j][i]*MtxB[2*j+1][i];
	}
	}
	*/
	
	return 0;
}

int Matrix::MtxParaMultiply(double *pMtxA, double MPParam, int m, int n, double *pMtxB) {
	for (int i = 0; i < m * n; i++) {
		*pMtxB = *pMtxA * MPParam;
		pMtxA++;
		pMtxB++;
	}
	return 0;
}

int Matrix::MtxSubtract(double *pMtxA, double *pMtxB, int m, int n, double *pMtxC) {
	for (int i = 0; i < m * n; i++) {
		*pMtxC = *pMtxA - *pMtxB;
		pMtxA++;
		pMtxB++;
		pMtxC++;
	}
	return 0;
}

int Matrix::MtxPlus(double *pMtxA, double *pMtxB, int m, int n, double *pMtxC) {
	for (int i = 0; i < m * n; i++) {
		*pMtxC = *pMtxA + *pMtxB;
		pMtxA++;
		pMtxB++;
		pMtxC++;
	}
	return 0;
}

