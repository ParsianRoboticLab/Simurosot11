// TParseString.h: interface for the TParseString class.
//
//////////////////////////////////////////////////////////////////////

#ifndef TPARSESTRING_HEADER
#define TPARSESTRING_HEADER

class TParseString : public CString {
public:
	TParseString() { m_iPosition = 0; };
	
	TParseString(LPCTSTR name) : CString(name) { m_iPosition = 0; };
	
	BOOL ParseBracket(TParseString &str);
	
	BOOL ParseString(CString &str);
	
	int ParseInteger();
	
	double ParseDouble();
	
	void OverreadLeadingSpaces();
	
	BOOL HasErrorOccured() const { return m_bErrorOccured; };
	
	int GetPosition() const { return m_iPosition; };
	
	void SetPosition(int pos);

private:
	int m_iPosition;
	BOOL m_bErrorOccured;
};

#endif // TPARSESTRING_HEADER
