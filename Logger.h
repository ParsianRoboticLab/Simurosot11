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
/*! \file Logger.h
<pre>
<b>File:</b>          Logger.h
<b>Project:</b>       Robocup Soccer Simulation Team: UvA Trilearn
<b>Authors:</b>       Jelle Kok
<b>Created:</b>       3/3/2000
<b>Last Revision:</b> $ID$
<b>Contents:</b>      This file contains the class to log information about the
system to any output stream. A range can be specified
for which the received log information is printed.
Furthermore it is possible to print the time since the
timer of the Logger has been restarted.
<hr size=2>
<h2><b>Changes</b></h2>
<b>Date</b>             <b>Author</b>          <b>Comment</b>
3/3/2001         Jelle Kok         Initial version created
27/11/2002       Alexey Vasilyev   Windows port
</pre>
*/

#ifndef _LOGGER_
#define _LOGGER_

#include "defines.h"

#define MAX_CYCLES 18000
#define DEFAULT_SPEED 100        // ms
#define SHOW_MODE        1
#define MSG_MODE        2

typedef struct {
	double x;
	double y;
	double angle;
} pos_t;

typedef struct {
	int time;
	pos_t pos[ROBOTNUMBER * 2 + 1];
} showinfo_t;

typedef struct {
	int time;
	int nTab;
	char message[128];
} msginfo_t;

typedef struct {
	short mode;
	union {
		showinfo_t show;
		msginfo_t msg;
	} body;
} dispinfo_t;


#include <stdio.h>            // needed for file

//#define OFFCLIENT

#ifndef OFFCLIENT
#else
#include "SimuroSotMonitorDoc.h"
#endif

#define MAX_LOG_LINE 8048 /*!< maximum size of a log message */
#define MAX_LOG_MONITOR 128
#define MAX_HEADER   64   /*!< maximum size of the header    */
#define MAX_LOG_LEVEL 32  /*!< maximun level of log          */

/*****************************************************************************/
/**************************** LOGGER *****************************************/
/*****************************************************************************/

/*! This class Makes it possible to log information on different abstraction
levels. All messages are passed to the log method 'log' with a level
indication. When it has been specified that this level should be logged
using either the 'addLogLevel' or 'addLogRange' method
the message is logged, otherwise it is ignored. This Makes it
possible to print only  the information you are interested in.
There is one global Log class which is used by all classes that use the
Logger. This instantation of the Logger is located in the file Logger.C and
is called 'Log'. All classes that want use this Logger should Make a
reference to it using the line 'extern Logger Log;' and can then use
this Logger with the Log.Log( ... ) methods. Furthermore the Logger also
contains a timer with Makes it possible to print the time since the timer
has been restarted. */
class Logger {
	char m_strBuf[5 * MAX_LOG_LINE];    /*!< buffer needed by different methods    */
	bool m_bLogLevels[MAX_LOG_LEVEL];  /*!< Set that contains all log levels      */
	char m_strHeader[MAX_LOG_LEVEL][MAX_HEADER];/*!< header string printed before msg      */
	FILE *m_fp;                   /*!< output stream to print messages to    */
	char m_strLogFileName[32];   /*!< log file name */
	int m_nLogBufferLen;                 /*!< log buffer len */

#ifndef OFFCLIENT
	COPYDATASTRUCT cpData;
	HWND hWnd;
#else
	CSimuroSotMonitorDoc*	m_pDoc;
#endif

public:
	Logger(char *strFileName);
	
	Logger();
	
	~Logger();
	
	// different methods associated with logging messages
	bool AssociateFile(char *strFileName);
	
	bool Log(int iLevel, char *str, ...);
	
	bool LogAction(int iLevel, char *str);
	
	bool IsInLogLevel(int iLevel);
	
	bool SetLogLevel(int iLevel, bool bSet = true);
	
	bool SetLogRange(int iMin, int iMax, bool bSet = true);
	
	char *GetHeader(int iLevel);
	
	bool SetHeader(int iLevel, char *str, bool bSet = true);
	
	FILE *GetLogFile();
	
	void SetLevelAndHeader();

/*******************************************************************/
	BOOL SendMessageOfGame(dispinfo_t *pInfo);
	
	BOOL GetDebugWindowHandle();
	
	void SendMsg(int timer, int nTab, char *msg);
	
	void Show(char *str, ...);
	
};


#endif
