// Matrix.h: interface for the Matrix class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MATRIX_H__15D773D0_E983_4E4B_853A_6F2759C02F0A__INCLUDED_)
#define AFX_MATRIX_H__15D773D0_E983_4E4B_853A_6F2759C02F0A__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class Matrix {
public:
	static int MtxMultiply(double *pMtxA, double *pMtxB, int m, int n, int q, double *pMtxC);
	
	static int MtxParaMultiply(double *pMtxA, double MPParam, int m, int n, double *pMtxB);
	
	static int MtxSubtract(double *pMtxA, double *pMtxB, int m, int n, double *pMtxC);
	
	static int MtxPlus(double *pMtxA, double *pMtxB, int m, int n, double *pMtxC);
};

#endif // !defined(AFX_MATRIX_H__15D773D0_E983_4E4B_853A_6F2759C02F0A__INCLUDED_)
