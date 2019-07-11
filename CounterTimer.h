// CounterTimer.h: interface for the CCounterTimer class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_COUNTERTIMER_H__AF65D259_9827_414A_A3AA_59C158D7FF77__INCLUDED_)
#define AFX_COUNTERTIMER_H__AF65D259_9827_414A_A3AA_59C158D7FF77__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/// Forward declaration
class CCounterTimer;

class CCounterTimerListener {
public:
	virtual void Update(CCounterTimer *Timer) = 0;
};


class CCounterTimer {
public:
	CCounterTimer();
	
	virtual ~CCounterTimer();
	
	/// Copying not allowed
	CCounterTimer(const CCounterTimer &);
	
	CCounterTimer &operator=(const CCounterTimer &);
	
	void AttachListener(CCounterTimerListener *pListener);
	
	void Step();
	
	void Stop();
	
	void Start(UINT nCount);

private:
	UINT m_nCount;
	BOOL m_bRunning;
	
	CCounterTimerListener *m_pListener;
};

#endif // !defined(AFX_COUNTERTIMER_H__AF65D259_9827_414A_A3AA_59C158D7FF77__INCLUDED_)
