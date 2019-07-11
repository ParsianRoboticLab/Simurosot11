// CounterTimer.cpp: implementation of the CCounterTimer class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "CounterTimer.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CCounterTimer::CCounterTimer()
		: m_nCount(0), m_bRunning(FALSE) {
	
}

CCounterTimer::~CCounterTimer() {

}

void CCounterTimer::Start(UINT nCount) {
	m_bRunning = TRUE;
	m_nCount = nCount;
}


void CCounterTimer::Stop() {
	m_bRunning = FALSE;
	m_nCount = 0;
}

void CCounterTimer::Step() {
	if (!m_bRunning || m_nCount == 0)
		return;
	
	m_nCount--;
	if (m_nCount == 0) {
		m_pListener->Update(this);
	}
}

void CCounterTimer::AttachListener(CCounterTimerListener *pListener) {
	m_pListener = pListener;
}
