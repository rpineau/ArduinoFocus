#include "x2focuser.h"


X2Focuser::X2Focuser(const char* pszDisplayName, 
												const int& nInstanceIndex,
												SerXInterface						* pSerXIn, 
												TheSkyXFacadeForDriversInterface	* pTheSkyXIn, 
												SleeperInterface					* pSleeperIn,
												BasicIniUtilInterface				* pIniUtilIn,
												LoggerInterface						* pLoggerIn,
												MutexInterface						* pIOMutexIn,
												TickCountInterface					* pTickCountIn)

{
	m_pSerX							= pSerXIn;		
	m_pTheSkyXForMounts				= pTheSkyXIn;
	m_pSleeper						= pSleeperIn;
	m_pIniUtil						= pIniUtilIn;
	m_pLogger						= pLoggerIn;	
	m_pIOMutex						= pIOMutexIn;
	m_pTickCount					= pTickCountIn;

	m_bLinked = false;
	m_nPosition = 0;
    m_fLastTemp = -273.15f; // aboslute zero :)

    // Read in settings
    if (m_pIniUtil) {
        m_ArduinoFocus.setPosLimit(m_pIniUtil->readInt(PARENT_KEY, POS_LIMIT, 999999)); // default for this focuser
        m_ArduinoFocus.enablePosLimit(m_pIniUtil->readInt(PARENT_KEY, POS_LIMIT_ENABLED, true));
        m_bContinous = m_pIniUtil->readInt(PARENT_KEY, CONTINUOUS_HOLD_ENABLED, false);
        m_bReversed = m_pIniUtil->readInt(PARENT_KEY, DIR_REVERSED, false);
    }
	m_ArduinoFocus.SetSerxPointer(m_pSerX);
	m_ArduinoFocus.setLogger(m_pLogger);
    m_ArduinoFocus.setSleeper(m_pSleeper);
}

X2Focuser::~X2Focuser()
{
    //Delete objects used through composition
	if (GetSerX())
		delete GetSerX();
	if (GetTheSkyXFacadeForDrivers())
		delete GetTheSkyXFacadeForDrivers();
	if (GetSleeper())
		delete GetSleeper();
	if (GetSimpleIniUtil())
		delete GetSimpleIniUtil();
	if (GetLogger())
		delete GetLogger();
	if (GetMutex())
		delete GetMutex();

}

#pragma mark - DriverRootInterface

int	X2Focuser::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LinkInterface_Name))
        *ppVal = (LinkInterface*)this;

    else if (!strcmp(pszName, FocuserGotoInterface2_Name))
        *ppVal = (FocuserGotoInterface2*)this;

    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);

    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);

    else if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();

    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);

    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);

    return SB_OK;
}

#pragma mark - DriverInfoInterface
void X2Focuser::driverInfoDetailedInfo(BasicStringInterface& str) const
{
        str = "Focuser X2 plugin by Rodolphe Pineau";
}

double X2Focuser::driverInfoVersion(void) const							
{
	return DRIVER_VERSION;
}

void X2Focuser::deviceInfoNameShort(BasicStringInterface& str) const
{
    str="ArduinoFocus";
}

void X2Focuser::deviceInfoNameLong(BasicStringInterface& str) const				
{
    deviceInfoNameShort(str);
}

void X2Focuser::deviceInfoDetailedDescription(BasicStringInterface& str) const		
{
	str = "ArduinoFocus Controller";
}

void X2Focuser::deviceInfoFirmwareVersion(BasicStringInterface& str)				
{
    if(!m_bLinked) {
        str="NA";
    }
    else {
        X2MutexLocker ml(GetMutex());
        // get firmware version
        char cFirmware[SERIAL_BUFFER_SIZE];
        m_ArduinoFocus.getFirmwareVersion(cFirmware, SERIAL_BUFFER_SIZE);
        str = cFirmware;
    }
}

void X2Focuser::deviceInfoModel(BasicStringInterface& str)							
{
    str="ArduinoFocus";
}

#pragma mark - LinkInterface
int	X2Focuser::establishLink(void)
{
    char szPort[DRIVER_MAX_STRING];
    int nErr;

    X2MutexLocker ml(GetMutex());
    // get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);
    nErr = m_ArduinoFocus.Connect(szPort);
    if(nErr) {
        m_bLinked = false;
        nErr = ERR_COMMOPENING;
    }
    else
        m_bLinked = true;

    m_ArduinoFocus.setContinuousHold(m_bContinous);
    m_ArduinoFocus.setRevereDir(m_bReversed);
    return nErr;
}

int	X2Focuser::terminateLink(void)
{
    if(!m_bLinked)
        return SB_OK;

    X2MutexLocker ml(GetMutex());
    m_ArduinoFocus.haltFocuser();
    m_ArduinoFocus.Disconnect();
    m_bLinked = false;

	return SB_OK;
}

bool X2Focuser::isLinked(void) const
{
	return m_bLinked;
}

#pragma mark - ModalSettingsDialogInterface
int	X2Focuser::initModalSettingsDialog(void)
{
    return SB_OK;
}

int	X2Focuser::execModalSettingsDialog(void)
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;
    bool bLimitEnabled = false;
    int nPosition = 0;
    int nPosLimit = 0;
    mUiEnabled = false;

    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("ArduinoFocus.ui", deviceType(), m_nPrivateMulitInstanceIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;

    X2MutexLocker ml(GetMutex());
	// set controls values
    if(m_bLinked) {
        // new position (set to current )
        nErr = m_ArduinoFocus.getPosition(nPosition);
        if(nErr)
            return nErr;
        dx->setEnabled("newPos", true);
        dx->setEnabled("pushButtonSet2", true);
        dx->setPropertyInt("newPos", "value", nPosition);
        dx->setEnabled("reverseDir", true);
        m_ArduinoFocus.getReverseDir(m_bReversed);
        dx->setChecked("reverseDir", m_bReversed);

        dx->setEnabled("continuousHold", true);
        m_ArduinoFocus.getContinuousHold(m_bContinous);
        dx->setChecked("continuousHold", m_bContinous);
}
    else {
        // disable all controls
        dx->setEnabled("newPos", false);
        dx->setPropertyInt("newPos", "value", 0);
        dx->setEnabled("reverseDir", false);
        dx->setEnabled("pushButtonSet2", false);
        dx->setEnabled("continuousHold", false);
    }

    // linit is done in software so it's always enabled.
    dx->setEnabled("posLimit", true);
    dx->setEnabled("limitEnable", true);
    dx->setPropertyInt("posLimit", "value", m_ArduinoFocus.getPosLimit());
    if(m_ArduinoFocus.isPosLimitEnabled())
        dx->setChecked("limitEnable", true);
    else
        dx->setChecked("limitEnable", false);



    //Display the user interface
    mUiEnabled = true;
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;
    mUiEnabled = false;

    //Retreive values from the user interface
    if (bPressedOK) {
        nErr = SB_OK;
        // get limit option
        bLimitEnabled = dx->isChecked("limitEnable");
        dx->propertyInt("posLimit", "value", nPosLimit);
        if(bLimitEnabled && nPosLimit>0) { // a position limit of 0 doesn't make sense :)
            m_ArduinoFocus.setPosLimit(nPosLimit);
            m_ArduinoFocus.enablePosLimit(bLimitEnabled);
        } else {
            m_ArduinoFocus.enablePosLimit(false);
        }
        m_bReversed = dx->isChecked("reverseDir");
        nErr |= m_ArduinoFocus.setRevereDir(m_bReversed);

        m_bContinous = dx->isChecked("continuousHold");
        nErr |= m_ArduinoFocus.setContinuousHold(m_bContinous);

        // save values to config
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, POS_LIMIT, nPosLimit);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, POS_LIMIT_ENABLED, bLimitEnabled);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, DIR_REVERSED, m_bReversed);
        nErr |= m_pIniUtil->writeInt(PARENT_KEY, CONTINUOUS_HOLD_ENABLED, m_bContinous);
    }
    return nErr;
}

void X2Focuser::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;
    int nTmpVal;
    char szErrorMessage[LOG_BUFFER_SIZE];

    // new position
    if (!strcmp(pszEvent, "on_pushButton_clicked")) {
        uiex->propertyInt("newPos", "value", nTmpVal);
        nErr = m_ArduinoFocus.syncMotorPosition(nTmpVal);
        if(nErr) {
            snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error setting new position : Error %d", nErr);
            uiex->messageBox("Set New Position", szErrorMessage);
            return;
        }
    }


}

#pragma mark - FocuserGotoInterface2
int	X2Focuser::focPosition(int& nPosition)
{
    int nErr;

    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());

    nErr = m_ArduinoFocus.getPosition(nPosition);
    m_nPosition = nPosition;
    return nErr;
}

int	X2Focuser::focMinimumLimit(int& nMinLimit) 		
{
	nMinLimit = 0;
    return SB_OK;
}

int	X2Focuser::focMaximumLimit(int& nPosLimit)			
{

	X2MutexLocker ml(GetMutex());
    if(m_ArduinoFocus.isPosLimitEnabled()) {
        nPosLimit = m_ArduinoFocus.getPosLimit();
    }
	else {
		nPosLimit = 100000;
	}

	return SB_OK;
}

int	X2Focuser::focAbort()								
{   int nErr;

    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());
    nErr = m_ArduinoFocus.haltFocuser();
    return nErr;
}

int	X2Focuser::startFocGoto(const int& nRelativeOffset)	
{
    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());
    m_ArduinoFocus.moveRelativeToPosision(nRelativeOffset);
    return SB_OK;
}

int	X2Focuser::isCompleteFocGoto(bool& bComplete) const
{
    int nErr;

    if(!m_bLinked)
        return NOT_CONNECTED;

    X2Focuser* pMe = (X2Focuser*)this;
    X2MutexLocker ml(pMe->GetMutex());
	nErr = pMe->m_ArduinoFocus.isGoToComplete(bComplete);

    return nErr;
}

int	X2Focuser::endFocGoto(void)
{
    int nErr;
    if(!m_bLinked)
        return NOT_CONNECTED;

    X2MutexLocker ml(GetMutex());
    nErr = m_ArduinoFocus.getPosition(m_nPosition);
    return nErr;
}

int X2Focuser::amountCountFocGoto(void) const					
{ 
	return 10;
}

int	X2Focuser::amountNameFromIndexFocGoto(const int& nZeroBasedIndex, BasicStringInterface& strDisplayName, int& nAmount)
{
	switch (nZeroBasedIndex)
	{
		default:
        case 0: strDisplayName="1 steps"; nAmount=10;break;
		case 1: strDisplayName="5 steps"; nAmount=100;break;
		case 2: strDisplayName="10 steps"; nAmount=1000;break;
        case 3: strDisplayName="50 steps"; nAmount=1000;break;
        case 4: strDisplayName="100 steps"; nAmount=1000;break;
        case 5: strDisplayName="500 steps"; nAmount=1000;break;
        case 6: strDisplayName="1000 steps"; nAmount=1000;break;
        case 7: strDisplayName="5000 steps"; nAmount=1000;break;
        case 8: strDisplayName="10000 steps"; nAmount=1000;break;
        case 9: strDisplayName="50000 steps"; nAmount=1000;break;
	}
	return SB_OK;
}

int	X2Focuser::amountIndexFocGoto(void)
{
	return 0;
}

#pragma mark - SerialPortParams2Interface

void X2Focuser::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Focuser::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort);

}


void X2Focuser::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize, DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);
    
}




