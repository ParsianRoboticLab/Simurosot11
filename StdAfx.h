// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently
//

#if !defined(AFX_STDAFX_H__801AABE7_2D67_11D4_A201_00400564C758__INCLUDED_)
#define AFX_STDAFX_H__801AABE7_2D67_11D4_A201_00400564C758__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define VC_EXTRALEAN		// Exclude rarely-used stuff from Windows headers

#include <afxwin.h>         // MFC core and standard components
#include <afxext.h>         // MFC extensions
#include <afxdisp.h>        // MFC Automation classes
#include <afxdtctl.h>		// MFC support for Internet Explorer 4 Common Controls
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>			// MFC support for Windows Common Controls
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <afxsock.h>		// MFC socket extensions
#include <math.h>
//#include "DSetupDlg.h"

#define LEFT_TO_RIGHT	0
#define RIGHT_TO_LEFT	1
#define MAX_PLAYER 3
#define OWN_TEAM 0
#define OTHER_TEAM 1

/*typedef struct {
	short	enable ;
	short	side ;
	short	unum ;
	short	angle ;
	short	x ;
	short	y ;
} pos_t ;*/

/*typedef struct{
		pos_t	pos[MAX_PLAYER * 2] ;

}send_to_team;*/


#define NO_OF_PLAYMODES		17
#define PLAYMODE_STRINGS {"",\
			"before_kick_off",\
			"time_over",\
			"play_on",\
			"kick_off_l",\
			"kick_off_r",\
			"kick_in_l",\
			"kick_in_r",\
			"free_kick_l",\
			"free_kick_r",\
			"corner_kick_l",\
			"corner_kick_r",\
			"goal_kick_l",\
			"goal_kick_r",\
			"goal_l",\
			"goal_r",\
			"drop_ball"}

typedef enum _PlayMode {
     PM_Null,
     PM_BeforeKickOff,
	 PM_TimeOver,
     PM_PlayOn,
     PM_KickOff_Left,
     PM_KickOff_Right,
     PM_KickIn_Left,
     PM_KickIn_Right,
	 PM_FreeKick_Left,
	 PM_FreeKick_Right,
	 PM_CornerKick_Left,
	 PM_CornerKick_Right,
	 PM_GoalKick_Left,
	 PM_GoalKick_Right,
	 PM_AfterGoal_Left,
	 PM_AfterGoal_Right,
	 PM_Drop_Ball,
     PM_MAX 
} PlayMode ;
//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_STDAFX_H__801AABE7_2D67_11D4_A201_00400564C758__INCLUDED_)
