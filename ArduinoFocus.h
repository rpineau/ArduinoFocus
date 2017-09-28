//
//  ArduinoFocus.h
//  NexDome
//
//  Created by Rodolphe Pineau on 2017/05/30.
//  NexDome X2 plugin

#ifndef __ArduinoFocus__
#define __ArduinoFocus__
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#include <string.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif
#ifdef SB_WIN_BUILD
#include <time.h>
#endif

#include <string>
#include <vector>
#include <sstream>
#include <iostream>

#include <exception>
#include <typeinfo>
#include <stdexcept>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"

// #define ArduinoFocus_DEBUG

#ifdef ArduinoFocus_DEBUG
#if defined(SB_WIN_BUILD)
#define ArduinoFocus_LOGFILENAME "C:\\ArduinoFocusLog.txt"
#elif defined(SB_LINUX_BUILD)
#define ArduinoFocus_LOGFILENAME "/tmp/ArduinoFocusLog.txt"
#elif defined(SB_MAC_BUILD)
#define ArduinoFocus_LOGFILENAME "/tmp/ArduinoFocusLog.txt"
#endif
#endif


#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 1000
#define LOG_BUFFER_SIZE 256

enum ArduinoFocus_Errors    {ArduinoFocus_OK = 0, NOT_CONNECTED, ND_CANT_CONNECT, ArduinoFocus_BAD_CMD_RESPONSE, COMMAND_FAILED};
enum MotorDir       {NORMAL = 0 , REVERSE};
enum MotorStatus    {IDLE = 0, MOVING};


class CArduinoFocus
{
public:
    CArduinoFocus();
    ~CArduinoFocus();

    int         Connect(const char *pszPort);
    void        Disconnect(void);
    bool        IsConnected(void) { return m_bIsConnected; };

    void        SetSerxPointer(SerXInterface *p) { m_pSerx = p; };
    void        setLogger(LoggerInterface *pLogger) { m_pLogger = pLogger; };
    void        setSleeper(SleeperInterface *pSleeper) { m_pSleeper = pSleeper; };

    // move commands
    int         haltFocuser();
    int         gotoPosition(int nPos);
    int         moveRelativeToPosision(int nSteps);

    // command complete functions
    int         isGoToComplete(bool &bComplete);
    int         isMotorMoving(bool &bMoving);

    // getter and setter
    void        setDebugLog(bool bEnable) {m_bDebugLog = bEnable; };

    int         getFirmwareVersionOnConnect();
    int         getFirmwareVersion(char *pszVersion, int nStrMaxLen);

    int         getPosition(int &nPosition);
    int         syncMotorPosition(int nPos);
    int         getPosLimit(void);
    void        setPosLimit(int nLimit);
    bool        isPosLimitEnabled(void);
    void        enablePosLimit(bool bEnable);
    int         setRevereDir(bool bReverse);
    void        getReverseDir(bool &bReverse);
    int         setContinuousHold(bool bContinous);
    void        getContinuousHold(bool &bContinous);
protected:

    int             ArduinoFocusCommand(const char *pszCmd, char *pszResult, int nResultMaxLen);
    int             readResponse(char *pszRespBuffer, int nBufferLen);
    int             parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator);

    SerXInterface   *m_pSerx;
    LoggerInterface *m_pLogger;
    SleeperInterface    *m_pSleeper;

    bool            m_bDebugLog;
    bool            m_bIsConnected;
    char            m_szFirmwareVersion[SERIAL_BUFFER_SIZE];
    char            m_szLogBuffer[LOG_BUFFER_SIZE];

    int             m_nCurPos;
    int             m_nTargetPos;
    int             m_nPosLimit;
    bool            m_bPosLimitEnabled;
    bool            m_bMoving;
    bool            m_bReverse;
    bool            m_bContinous;

#ifdef ArduinoFocus_DEBUG
	// timestamp for logs
	char *timestamp;
	time_t ltime;
	FILE *Logfile;	  // LogFile
#endif

};

#endif //__ArduinoFocus__
