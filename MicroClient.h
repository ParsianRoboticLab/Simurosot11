// MicroClient.h : main header file for the MICROCLIENT application
//

#ifndef MICROCLIENT_HEADER
#define MICROCLIENT_HEADER


#ifndef __AFXWIN_H__
#error include 'stdafx.h' before including this file for PCH
#endif

#include "resource.h"       // main symbols

/////////////////////////////////////////////////////////////////////////////
// CMicroClientApp:
// See MicroClient.cpp for the implementation of this class
//

class CMicroClientApp : public CWinApp {
public:
	CMicroClientApp();

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CMicroClientApp)
public:
	virtual BOOL InitInstance();
	//}}AFX_VIRTUAL

// Implementation
	//{{AFX_MSG(CMicroClientApp)
	afx_msg void OnAppAbout();
	
	// NOTE - the ClassWizard will add and remove member functions here.
	//    DO NOT EDIT what you see in these blocks of generated code !
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

#endif // MICROCLIENT_HEADER
