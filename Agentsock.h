#ifndef __AGENTSOCK_H__
#define __AGENTSOCK_H__

class CAgentSocket : public CAsyncSocket {
	DECLARE_DYNAMIC(CAgentSocket);

public:
	CAgentSocket();
	
	// functions from CAsyncsocket adapted for this use
	BOOL Create(UINT nSocketPort, LPCTSTR lpszSocketAddress);
	
	BOOL Bind(UINT nSocketPort, LPCTSTR lpszSocketAddress);
	
	BOOL Connect(LPCTSTR lpszHostAddress, UINT nHostPort);
	
	int ReceiveFrom(void *lpBuf, int nBufLen, CString &rSocketAddress, UINT &rSocketPort, int nFlags);
	
	int SendTo(const void *lpBuf, int nBufLen, UINT nHostPort, LPCTSTR lpszHostAddress, int nFlags);
	
	// get and set functions
	void SetPort(UINT port) { m_uiPort = port; };
	
	UINT GetPort(void) { return m_uiPort; };
	
	CString GetAddress(void) { return m_sSocketAddress; };

protected:
	// notification functions
	virtual void OnReceive(int nErrorCode);

private:
	SOCKADDR_IN sockAddr;
	UINT m_uiPort;
	CString m_sSocketAddress;
};

#endif // __AGENTSOCK_H__
