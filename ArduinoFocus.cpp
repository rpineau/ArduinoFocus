//
//  nexdome.cpp
//  NexDome X2 plugin
//
//  Created by Rodolphe Pineau on 6/11/2016.


#include "ArduinoFocus.h"



CArduinoFocus::CArduinoFocus()
{

    m_pSerx = NULL;
    m_pLogger = NULL;


    m_bDebugLog = false;
    m_bIsConnected = false;

    m_nCurPos = 0;
    m_nTargetPos = 0;
    m_nPosLimit = 999999; // default for this focuser
    m_bPosLimitEnabled = true;
    m_bMoving = false;
    m_bReverse = false;
    m_bContinous = false;
    
#ifdef	ArduinoFocus_DEBUG
	Logfile = fopen(ArduinoFocus_LOGFILENAME, "w");
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CArduinoFocus Constructor Called.\n", timestamp);
	fflush(Logfile);
#endif

}

CArduinoFocus::~CArduinoFocus()
{
#ifdef	ArduinoFocus_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif
}

int CArduinoFocus::Connect(const char *pszPort)
{
    int nErr = ArduinoFocus_OK;

    if(!m_pSerx)
        return ERR_COMMNOLINK;

#ifdef ArduinoFocus_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CArduinoFocus::Connect Called %s\n", timestamp, pszPort);
	fflush(Logfile);
#endif

    // 19200 8N1
    if(m_pSerx->open(pszPort, 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

    m_pSleeper->sleep(2000);

#ifdef ArduinoFocus_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CArduinoFocus::Connect connected to %s\n", timestamp, pszPort);
	fflush(Logfile);
#endif
	
    if (m_bDebugLog && m_pLogger) {
        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CArduinoFocus::Connect] Connected.\n");
        m_pLogger->out(m_szLogBuffer);

        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CArduinoFocus::Connect] Getting Firmware.\n");
        m_pLogger->out(m_szLogBuffer);
    }
    getFirmwareVersionOnConnect();
    return nErr;
}

void CArduinoFocus::Disconnect()
{
    if(m_bIsConnected && m_pSerx)
        m_pSerx->close();
 
	m_bIsConnected = false;
}

#pragma mark move commands
int CArduinoFocus::haltFocuser()
{
    int nErr;
    char szResp[SERIAL_BUFFER_SIZE];
    int nPos;

	if(!m_bIsConnected)
		return NOT_CONNECTED;

    nErr = ArduinoFocusCommand("H#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    if(!strstr(szResp,"H#"))
        return ArduinoFocus_BAD_CMD_RESPONSE;

    getPosition(nPos);
    m_nCurPos = nPos;
    m_nTargetPos = m_nCurPos;

    return nErr;
}

int CArduinoFocus::gotoPosition(int nPos)
{
    int nErr;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFields;
    std::vector<std::string> vFieldsData;

    if(!m_bIsConnected)
		return NOT_CONNECTED;

    if (m_bPosLimitEnabled && nPos>m_nPosLimit)
        return ERR_LIMITSEXCEEDED;

#ifdef ArduinoFocus_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CArduinoFocus::gotoPosition goto position  : %d\n", timestamp, nPos);
    fflush(Logfile);
#endif

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "M %d#", nPos);
    nErr = ArduinoFocusCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // parse output to extract position value.
    nErr = parseFields(szResp, vFields, ';');
    // convert response
    // position
    if(vFields.size()>=1) {
        nErr = parseFields(vFields[0].c_str(), vFieldsData, ' ');
        if(vFieldsData.size()==2) {
            m_nTargetPos = atoi(vFieldsData[1].c_str());
        }
    }
    else
        nErr = ArduinoFocus_BAD_CMD_RESPONSE;

    return nErr;
}

int CArduinoFocus::moveRelativeToPosision(int nSteps)
{
    int nErr;

	if(!m_bIsConnected)
		return NOT_CONNECTED;

#ifdef ArduinoFocus_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CArduinoFocus::gotoPosition goto relative position  : %d\n", timestamp, nSteps);
    fflush(Logfile);
#endif

    m_nTargetPos = m_nCurPos + nSteps;
    nErr = gotoPosition(m_nTargetPos);
    return nErr;
}

#pragma mark command complete functions

int CArduinoFocus::isGoToComplete(bool &bComplete)
{
    int nErr = ArduinoFocus_OK;
	
	if(!m_bIsConnected)
		return NOT_CONNECTED;

    getPosition(m_nCurPos);
    if(m_nCurPos == m_nTargetPos)
        bComplete = true;
    else
        bComplete = false;
    return nErr;
}

int CArduinoFocus::isMotorMoving(bool &bMoving)
{
    int nErr = ArduinoFocus_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFields;
    std::vector<std::string> vFieldsData;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = ArduinoFocusCommand("G#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // parse output to extract position value.
    nErr = parseFields(szResp, vFields, ';');
    // convert response
    if(vFields.size()>=1) {
        // position
        nErr = parseFields(vFields[0].c_str(), vFieldsData, ' ');
        if(vFieldsData.size()==2) {
            m_nCurPos = atoi(vFieldsData[1].c_str());
        }

        // moving ?
        nErr = parseFields(vFields[1].c_str(), vFieldsData, ' ');
        if(vFieldsData.size()==2) {
            if(vFieldsData[1] == "false")
                bMoving = false;
            else
                bMoving = true;
        }
        m_bMoving = bMoving;
    }
    else
        nErr = ArduinoFocus_BAD_CMD_RESPONSE;

    return nErr;
}

#pragma mark getters and setters

int CArduinoFocus::getFirmwareVersionOnConnect()
{
    int nErr = ArduinoFocus_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFields;
    std::vector<std::string> vFieldsData;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
#ifdef ArduinoFocus_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CArduinoFocus::getFirmwareVersionOnConnect szResp : %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    // parse response.
    nErr = parseFields(szResp, vFields, '\n');
    if(vFields.size()>=2) {
        // firmware version
        strncpy(m_szFirmwareVersion, vFields[0].c_str(), SERIAL_BUFFER_SIZE);
        // reverse
        nErr = parseFields(vFields[1].c_str(), vFieldsData, '=');
        if(vFieldsData.size()>=2) {
            if(strstr(vFieldsData[1].c_str(),"0#"))
                m_bReverse = true;
            else
                m_bReverse = false;

        }
    }

    return nErr;
}

int CArduinoFocus::getFirmwareVersion(char *pszVersion, int nStrMaxLen)
{
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    strncpy(pszVersion, m_szFirmwareVersion, nStrMaxLen);
    return ArduinoFocus_OK;
}


int CArduinoFocus::getPosition(int &nPosition)
{
    int nErr = ArduinoFocus_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFields;
    std::vector<std::string> vFieldsData;

	if(!m_bIsConnected)
		return NOT_CONNECTED;

    nErr = ArduinoFocusCommand("G#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // parse output to extract position value.
    nErr = parseFields(szResp, vFields, ';');
    // convert response
    // position
    nErr = parseFields(vFields[0].c_str(), vFieldsData, ' ');
    if(vFieldsData.size()==2) {
        nPosition = atoi(vFieldsData[1].c_str());
    }

    m_nCurPos = nPosition;

    // moving ?
    nErr = parseFields(vFields[1].c_str(), vFieldsData, ' ');
    if(vFieldsData.size()==2) {
        if(vFieldsData[1] == "false")
            m_bMoving = false;
        else
            m_bMoving = true;
    }

    return nErr;
}


int CArduinoFocus::syncMotorPosition(int nPos)
{
    int nErr = ArduinoFocus_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFields;
    std::vector<std::string> vFieldsData;

	if(!m_bIsConnected)
		return NOT_CONNECTED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "P %d#", nPos);
    nErr = ArduinoFocusCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // parse output to extract position value.
    nErr = parseFields(szResp, vFields, ';');
    // convert response
    // position
    nErr = parseFields(vFields[0].c_str(), vFieldsData, ' ');
    if(vFieldsData.size()==2) {
        m_nCurPos = atoi(vFieldsData[1].c_str());
    }
    return nErr;
}



int CArduinoFocus::getPosLimit()
{
    return m_nPosLimit;
}

void CArduinoFocus::setPosLimit(int nLimit)
{
    m_nPosLimit = nLimit;
}

bool CArduinoFocus::isPosLimitEnabled()
{
    return m_bPosLimitEnabled;
}

void CArduinoFocus::enablePosLimit(bool bEnable)
{
    m_bPosLimitEnabled = bEnable;
}

int CArduinoFocus::setRevereDir(bool bReverse)
{
    int nErr = ArduinoFocus_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFields;
    std::vector<std::string> vFieldsData;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "R %d#", bReverse?1:0);

    nErr = ArduinoFocusCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // parse output to extract position value.
    nErr = parseFields(szResp, vFields, '=');
    if(nErr)
        return nErr;
    if(vFields.size()>=1) {
        if(strstr(vFields[1].c_str(),"0"))
            m_bReverse = false;
        else
            m_bReverse = true;
    }
    return nErr;
}

void CArduinoFocus::getReverseDir(bool &bReverse)
{
    bReverse = m_bReverse;
}

int CArduinoFocus::setContinuousHold(bool bContinous)
{
    int nErr = ArduinoFocus_OK;
    char szCmd[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> vFields;
    std::vector<std::string> vFieldsData;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szCmd, SERIAL_BUFFER_SIZE, "C %d#", bContinous?1:0);

    nErr = ArduinoFocusCommand(szCmd, szResp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // parse output to extract position value.
    nErr = parseFields(szResp, vFields, '=');
    if(nErr)
        return nErr;
    if(vFields.size()>=1) {
        if(strstr(vFields[1].c_str(),"0")) {
            m_bContinous = false;
        }
        else
            m_bContinous = true;
    }
    return nErr;

}

void CArduinoFocus::getContinuousHold(bool &bContinous)
{
    bContinous = m_bContinous;
}

#pragma mark command and response functions

int CArduinoFocus::ArduinoFocusCommand(const char *pszszCmd, char *pszResult, int nResultMaxLen)
{
    int nErr = ArduinoFocus_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;
	
	if(!m_bIsConnected)
		return NOT_CONNECTED;

    m_pSerx->purgeTxRx();
    if (m_bDebugLog && m_pLogger) {
        snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CArduinoFocus::ArduinoFocusCommand] Sending %s\n",pszszCmd);
        m_pLogger->out(m_szLogBuffer);
    }
#ifdef ArduinoFocus_DEBUG
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] CArduinoFocus::ArduinoFocusCommand Sending %s\n", timestamp, pszszCmd);
	fflush(Logfile);
#endif
    nErr = m_pSerx->writeFile((void *)pszszCmd, strlen(pszszCmd), ulBytesWrite);
    m_pSerx->flushTx();

    if(nErr){
        if (m_bDebugLog && m_pLogger) {
            snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CArduinoFocus::ArduinoFocusCommand] writeFile Error.\n");
            m_pLogger->out(m_szLogBuffer);
        }
        return nErr;
    }

    if(pszResult) {
        // read response
        if (m_bDebugLog && m_pLogger) {
            snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CArduinoFocus::ArduinoFocusCommand] Getting response.\n");
            m_pLogger->out(m_szLogBuffer);
        }
        nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
        if(nErr){
            if (m_bDebugLog && m_pLogger) {
                snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CArduinoFocus::ArduinoFocusCommand] readResponse Error.\n");
                m_pLogger->out(m_szLogBuffer);
            }
        }
#ifdef ArduinoFocus_DEBUG
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] CArduinoFocus::ArduinoFocusCommand response \"%s\"\n", timestamp, szResp);
		fflush(Logfile);
#endif
        // printf("Got response : %s\n",resp);
        strncpy(pszResult, szResp, nResultMaxLen);
#ifdef ArduinoFocus_DEBUG
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] CArduinoFocus::ArduinoFocusCommand response copied to pszResult : \"%s\"\n", timestamp, pszResult);
		fflush(Logfile);
#endif
    }
    return nErr;
}

int CArduinoFocus::readResponse(char *pszRespBuffer, int nBufferLen)
{
    int nErr = ArduinoFocus_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;
	
	if(!m_bIsConnected)
		return NOT_CONNECTED;

    memset(pszRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = pszRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, MAX_TIMEOUT);
        if(nErr) {
            if (m_bDebugLog && m_pLogger) {
                snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CArduinoFocus::readResponse] readFile Error.\n");
                m_pLogger->out(m_szLogBuffer);
            }
            return nErr;
        }

        if (ulBytesRead !=1) {// timeout
            if (m_bDebugLog && m_pLogger) {
                snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CArduinoFocus::readResponse] readFile Timeout.\n");
                m_pLogger->out(m_szLogBuffer);
            }
#ifdef ArduinoFocus_DEBUG
			ltime = time(NULL);
			timestamp = asctime(localtime(&ltime));
			timestamp[strlen(timestamp) - 1] = 0;
			fprintf(Logfile, "[%s] CArduinoFocus::readResponse timeout\n", timestamp);
			fflush(Logfile);
#endif
            nErr = ERR_NORESPONSE;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
        if (m_bDebugLog && m_pLogger) {
            snprintf(m_szLogBuffer,LOG_BUFFER_SIZE,"[CArduinoFocus::readResponse] ulBytesRead = %lu\n",ulBytesRead);
            m_pLogger->out(m_szLogBuffer);
        }
    } while (*pszBufPtr++ != '#' && ulTotalBytesRead < nBufferLen );

    if(ulTotalBytesRead)
        *(pszBufPtr-1) = 0; //remove the #

    return nErr;
}

int CArduinoFocus::parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = ArduinoFocus_OK;
    std::string sSegment;
    std::stringstream ssTmp(pszResp);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}

