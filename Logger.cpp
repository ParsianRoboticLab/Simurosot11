/*
Copyright (c) 2000-2002, Jelle Kok, University of Amsterdam
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this 
list of conditions and the following disclaimer. 

2. Redistributions in binary form must reproduce the above copyright notice, 
this list of conditions and the following disclaimer in the documentation 
and/or other materials provided with the distribution. 

3. Neither the name of the University of Amsterdam nor the names of its 
contributors may be used to endorse or promote products derived from this 
software without specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE 
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*! \file Logger.C
<pre>
<b>File:</b>          Logger.C
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       3/3/2000
<b>Last Revision:</b> $ID$
<b>Contents:</b>      This file contains the definitions for the class to log
               information about the system to any output stream. A
               range can be specified for which the received log
               information is printed. Furthermore it is possible to
               print the time since the timer of the Logger has been
               restarted.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
3/3/2001         Jelle Kok         Initial version created
27/11/2002       Alexey Vasilyev   Windows port
</pre>
*/
#include "stdafx.h"
#include "Logger.h"

#include <string.h>		// needed for string operation, such as strcpy
#ifdef Solaris
  #include <varargs.h> // needed for va_list and va_start under Solaris
#else
  #include <stdarg.h>
#endif

#ifdef OFFCLIENT
extern CSimuroSotMonitorApp theApp;
#endif
/*****************************************************************************/
/**************************** LOGGER *****************************************/
/*****************************************************************************/

/*!This is the constructor for the Logger. The output stream, the minimal and
   maximal log level can all be specified. The timer in this class is also
   restarted.
   \param FileName file name to which information is printed
   \param iMin minimal log level (default 0)
   \param iMax maximal log level (default 0) */
Logger::Logger(  char* strFileName)
{
/*	SetLogRange(0, MAX_LOG_LEVEL, false);
	for (int i=0; i<MAX_LOG_LEVEL; i++)
	{
		strcpy(m_strHeader[i], "");
	}

	m_nLogBufferLen = 0;

	AssociateFile(strFileName);

	SetLevelAndHeader();*/
}

Logger::Logger()
{
	SetLogRange(0, MAX_LOG_LEVEL, false);
	for (int i=0; i<MAX_LOG_LEVEL; i++)
	{
		strcpy(m_strHeader[i], "");
	}
	m_nLogBufferLen = 0;

	m_fp = NULL;

	SetLevelAndHeader();

#ifndef OFFCLIENT
#else
	// get the document
	POSITION pos = theApp.GetFirstDocTemplatePosition();
	CDocTemplate* pDoc = theApp.GetNextDocTemplate(pos);
	pos = pDoc->GetFirstDocPosition();
	m_pDoc = (CSimuroSotMonitorDoc*) pDoc->GetNextDoc(pos);
#endif

}

Logger::~Logger()
{
	if (m_fp != NULL)
	{
		if (m_nLogBufferLen > 0)
		{
			fwrite(m_strBuf, m_nLogBufferLen, 1, m_fp);
			fflush(m_fp);
			m_nLogBufferLen = 0;
		}
		fclose(m_fp);
	}
}

/*!This method is used to associate the log class to log file. That is to
say, the log informations is saved in this file.
\param strFileName is the log file name
\return bool indicating whether the file is opened or not. */
bool Logger::AssociateFile(char* strFileName)
{
/*	if (strFileName != NULL)
	{
		strcpy(m_strLogFileName, strFileName);
		m_fp = fopen(m_strLogFileName, "w");
		if (m_fp!= NULL)
		{
			return true;
		}
	}
*/	return false;
}
/*!This method can be used to log information. Only when the specified
level of the message is part of the Set of logged values the
information is logged. This method receives a character string that may
contain format specifiers that are also available to 'printf' (like %d, %5.2f,
etc.). The remaining arguments are the variables that have to be filled in
at the location of the specifiers.
\param iLevel level corresponding to this message
\param str character string with (possible) format specifiers
\return bool indicating whether the message was logged or not. */
bool Logger::LogAction(int iLevel, char* str)
{

	if (iLevel < 0 || iLevel >= MAX_LOG_LEVEL) iLevel = 0;
	if (!IsInLogLevel(iLevel)) return false;

	if (m_fp != NULL)
	{
		int nLenHeader, nLenLog;
		char strHeader[100];

		if (strcmp(m_strHeader[iLevel], ""))
		nLenHeader = sprintf(strHeader,"[%s]", m_strHeader[iLevel]);
		else
		nLenHeader = sprintf(strHeader,"");

		nLenLog = strlen(str);

		// if log buffer is full, then write to file
		if (m_nLogBufferLen + nLenHeader + nLenLog + 1 > 1/*5 * MAX_LOG_LINE - 100*/
		&& m_nLogBufferLen > 0)
		{
			fwrite(m_strBuf, m_nLogBufferLen, 1, m_fp);
			fflush(m_fp);
			m_nLogBufferLen = 0;
		}

		// copy the log string to log buffer
		memcpy(m_strBuf+m_nLogBufferLen, strHeader, nLenHeader);
		m_nLogBufferLen += nLenHeader;
		memcpy(m_strBuf+m_nLogBufferLen, str, nLenLog);
		m_nLogBufferLen += nLenLog;
		*(m_strBuf+m_nLogBufferLen) = '\n';	// 10 is the end Mark
		m_nLogBufferLen += 1;

		return true;
	}
	else
	{
		return false;
	}
}

/*!This method can be used to log information. Only when the specified
   level of the message is part of the Set of logged values the
   information is logged. This method receives a character string that may
   contain format specifiers that are also available to 'printf' (like %d, %5.2f,
   etc.). The remaining arguments are the variables that have to be filled in
   at the location of the specifiers.
   \param iLevel level corresponding to this message
   \param str character string with (possible) format specifiers
   \param ... variables that define the values of the specifiers.
   \return bool indicating whether the message was logged or not. */
bool Logger::Log( int iLevel, char *str, ... )
{
  if( IsInLogLevel( iLevel ) )
  {
	char strLogString[2*MAX_LOG_LINE];
    va_list ap;
#ifdef Solaris
    va_start( ap );
#else
    va_start( ap, str );
#endif
    vsprintf( strLogString, str, ap );
    va_end(ap);

    return LogAction(iLevel, strLogString);
  }
  return false;
}



/*!This method returns whether the supplied log level is recorded, thus whether
   it is part of the Set of logged levels.
   \param iLevel log level that should be checked
   \return bool indicating whether the supplied log level is logged. */
bool Logger::IsInLogLevel( int iLevel )
{
		if (iLevel < 0 || iLevel >= MAX_LOG_LEVEL) iLevel = 0;
  return m_bLogLevels[iLevel];
}

/*! This method inserts the log level 'iLevel' to the Set of logged levels. 
    Information from this log level will be printed. 
    \param iLevel level that will be added to the Set
    \return bool indicating whether the Update was successfull. */
bool  Logger::SetLogLevel( int iLevel, bool bSet )
{
	if (iLevel >= 0 && iLevel < MAX_LOG_LEVEL)
	{
		m_bLogLevels[iLevel] = bSet;
		return true;
	}
	else
	{
		return false;
	}
}

/*! This method inserts all the log levels in the interval [iMin..iMax] to
    the Set of logged levels. 
    \param iMin minimum log level that is added
    \param iMax maximum log level that is added 
    \return bool indicating whether the Update was successfull. */
bool  Logger::SetLogRange( int iMin, int iMax, bool bSet )
{
	  bool bReturn = true;
	  for( int i = iMin ; i < iMax;  i++ )
		bReturn &= SetLogLevel( i, bSet );
	  return bReturn;
}

/*! This method returns the current header that is written before the actual
    text that has to be logged.
    \return current header */
char* Logger::GetHeader( int iLevel )
{
	if (iLevel >= 0 && iLevel < MAX_LOG_LEVEL)
	{
		return m_strHeader[iLevel];
	}
	else
	{
		return false;
	}
}

/*! This method Sets the header that is written before the actual logging text.
		\param iLevel that represents the log level you want to Set header
    \param str that represents the character string
    \return bool indicating whether the Update was succesfull */
bool Logger::SetHeader(int iLevel, char *str, bool bSet)
{
	if (iLevel >= 0 && iLevel < MAX_LOG_LEVEL)
	{			
	
		strcpy( &m_strHeader[iLevel][0], str );
		SetLogLevel(iLevel, bSet);
		return true;
	}
	else
	{
		return false;
	}
}


/*!This method returns the output stream to which the log information is 
   written. This outputstream can be standard output (cout) or a reference to a
   file.
   \return o outputstream to which log information is printed. */
FILE* Logger::GetLogFile(  )
{
  return m_fp;
}

void Logger::SetLevelAndHeader()
{
	// set the log headers defined before.
//	SetHeader(0, "Error : ");
//	SetHeader(1, "Connection : ");
//	SetHeader(2, "Act Handle : ");
//	SetHeader(3, "Sense Handle : ");
//	SetHeader(4, "Synchronize : ");
//	SetHeader(10, "Infomation Update : ");
//	SetHeader(11, "Partical Process : ");
//	SetHeader(12, "High Level : ");
//	SetHeader(15, "Actions : ");
//	SetHeader(20, "Skills : ");
	SetHeader(25, "Strategy : ");
//	SetHeader(31, "Debug : ");

	//	SetLogLevel(30);		// this is for dos show

}


void Logger::Show(char *str, ...)
{
	char strLogString[2*MAX_LOG_LINE];
	va_list ap;
	va_start( ap, str );
	vsprintf( strLogString, str, ap );
	va_end(ap);
		
	MessageBox(NULL, strLogString, "Log", MB_OK);
}


void Logger::SendMsg(int timer, int nTab, char *msg)
{
	dispinfo_t dispinfo;
	dispinfo.mode = MSG_MODE;
	dispinfo.body.msg.time = timer;
	dispinfo.body.msg.nTab = nTab;
	
	sprintf( dispinfo.body.msg.message, msg);
	
#ifndef OFFCLIENT
	SendMessageOfGame(&dispinfo);
#else
	m_pDoc->m_pLogPlayer->AddLogInfo(&dispinfo.body.msg);
#endif	
}

BOOL Logger::GetDebugWindowHandle()
{
	//find the Debug window
	hWnd = ::FindWindow("SimuroSot Log Player", NULL);
	if(!hWnd)
		return FALSE;
	else
		return TRUE;
}

BOOL Logger::SendMessageOfGame(dispinfo_t *pInfo)
{
	if(GetDebugWindowHandle() == FALSE) return FALSE;

	//set up a COPYDATASTRUCT structure for use with WM_COPYDATA
	cpData.dwData = (DWORD) 999;
	cpData.cbData = sizeof(dispinfo_t);
	cpData.lpData = pInfo;
	
	//send the structure to Debug via the system
	LRESULT lResult = ::SendMessage(hWnd, WM_COPYDATA, (UINT) NULL, (long) &cpData);
//	LRESULT lResult = ::PostMessage(hWnd, WM_COPYDATA, (UINT) NULL, (long) &cpData);

	if((BOOL) lResult == TRUE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
