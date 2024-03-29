// X2Focuser.h : Declaration of the X2Focuser

#ifndef __X2Focuser_H_
#define __X2Focuser_H_

#include <stdio.h>
#include <string.h>

#include "../../licensedinterfaces/theskyxfacadefordriversinterface.h"
#include "../../licensedinterfaces/modalsettingsdialoginterface.h"
#include "../../licensedinterfaces/x2guiinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/basiciniutilinterface.h"
#include "../../licensedinterfaces/mutexinterface.h"
#include "../../licensedinterfaces/basicstringinterface.h"
#include "../../licensedinterfaces/tickcountinterface.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/focuserdriverinterface.h"
#include "../../licensedinterfaces/serialportparams2interface.h"
#include "../../licensedinterfaces/sberrorx.h"


#include "StopWatch.h"
#include "ArduinoFocus.h"

// Forward declare the interfaces that this device is dependent upon
class SerXInterface;
class TheSkyXFacadeForDriversInterface;
class SleeperInterface;
class SimpleIniUtilInterface;
class LoggerInterface;
class MutexInterface;
class BasicIniUtilInterface;
class TickCountInterface;

#define DRIVER_VERSION      1.1

#define PARENT_KEY			"ArduinoFocus"
#define CHILD_KEY_PORTNAME	"PortName"
#define POS_LIMIT           "PosLimit"
#define POS_LIMIT_ENABLED   "PosLimitEnable"
#define DIR_REVERSED        "DirReversed"
#define CONTINUOUS_HOLD_ENABLED   "ContinuousHoldEnable"

#if defined(SB_WIN_BUILD)
#define DEF_PORT_NAME					"COM1"
#elif defined(SB_MAC_BUILD)
#define DEF_PORT_NAME					"/dev/cu.KeySerial1"
#elif defined(SB_LINUX_BUILD)
#define DEF_PORT_NAME					"/dev/ttyUSB0"
#endif

#define LOG_BUFFER_SIZE 256
#define TMP_BUF_SIZE    1024

/*!
\brief The X2Focuser example.

\ingroup Example

Use this example to write an X2Focuser driver.
*/
class X2Focuser : public FocuserDriverInterface, public SerialPortParams2Interface, public ModalSettingsDialogInterface, public X2GUIEventInterface
{
public:
	X2Focuser(const char                        *pszDisplayName,
            const int                           &nInstanceIndex,
            SerXInterface						* pSerXIn, 
            TheSkyXFacadeForDriversInterface	* pTheSkyXIn, 
            SleeperInterface					* pSleeperIn,
            BasicIniUtilInterface				* pIniUtilIn,
            LoggerInterface						* pLoggerIn,
            MutexInterface						* pIOMutexIn,
            TickCountInterface					* pTickCountIn);

	~X2Focuser();

public:

	/*!\name DriverRootInterface Implementation
	See DriverRootInterface.*/
	//@{ 
	virtual int                                 queryAbstraction(const char* pszName, void** ppVal);
	//@} 

	/*!\name DriverInfoInterface Implementation
	See DriverInfoInterface.*/
	//@{ 
	virtual void								driverInfoDetailedInfo(BasicStringInterface& str) const;
	virtual double								driverInfoVersion(void) const;
	//@} 

	/*!\name HardwareInfoInterface Implementation
	See HardwareInfoInterface.*/
	//@{ 
	virtual void                                deviceInfoNameShort(BasicStringInterface& str) const;
	virtual void                                deviceInfoNameLong(BasicStringInterface& str) const;
	virtual void                                deviceInfoDetailedDescription(BasicStringInterface& str) const;
	virtual void                                deviceInfoFirmwareVersion(BasicStringInterface& str);
	virtual void                                deviceInfoModel(BasicStringInterface& str);
	//@} 

	/*!\name LinkInterface Implementation
	See LinkInterface.*/
	//@{ 
	virtual int									establishLink(void);
	virtual int									terminateLink(void);
	virtual bool								isLinked(void) const;
	//@} 

    // ModalSettingsDialogInterface
    virtual int                                 initModalSettingsDialog(void);
    virtual int                                 execModalSettingsDialog(void);
    virtual void                                uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent);

	/*!\name FocuserGotoInterface2 Implementation
	See FocuserGotoInterface2.*/
	virtual int									focPosition(int& nPosition) 			;
	virtual int									focMinimumLimit(int& nMinLimit) 		;
	virtual int									focMaximumLimit(int& nMaxLimit)			;
	virtual int									focAbort()								;

	virtual int                                 startFocGoto(const int& nRelativeOffset)	;
	virtual int                                 isCompleteFocGoto(bool& bComplete) const	;
	virtual int                                 endFocGoto(void)							;

	virtual int                                 amountCountFocGoto(void) const				;
	virtual int                                 amountNameFromIndexFocGoto(const int& nZeroBasedIndex, BasicStringInterface& strDisplayName, int& nAmount);
	virtual int                                 amountIndexFocGoto(void)					;
	//@}

    //SerialPortParams2Interface
    virtual void                                portName(BasicStringInterface& str) const			;
    virtual void                                setPortName(const char* pszPort)						;
    virtual unsigned int                        baudRate() const			{return 9600;};
    virtual void                                setBaudRate(unsigned int)	{};
    virtual bool                                isBaudRateFixed() const		{return true;}

    virtual SerXInterface::Parity               parity() const				{return SerXInterface::B_NOPARITY;}
    virtual void                                setParity(const SerXInterface::Parity& parity){parity;};
    virtual bool                                isParityFixed() const		{return true;}


private:

    int                                     m_nPrivateMulitInstanceIndex;

	//Standard device driver tools
	SerXInterface*							m_pSerX;		
	TheSkyXFacadeForDriversInterface* 		m_pTheSkyXForMounts;
	SleeperInterface*						m_pSleeper;
	BasicIniUtilInterface*					m_pIniUtil;
	LoggerInterface*						m_pLogger;
	mutable MutexInterface*					m_pIOMutex;
	TickCountInterface*						m_pTickCount;

	SerXInterface 							*GetSerX() {return m_pSerX; }		
	TheSkyXFacadeForDriversInterface		*GetTheSkyXFacadeForDrivers() {return m_pTheSkyXForMounts;}
	SleeperInterface						*GetSleeper() {return m_pSleeper; }
	BasicIniUtilInterface					*GetSimpleIniUtil() {return m_pIniUtil; }
	LoggerInterface							*GetLogger() {return m_pLogger; }
	MutexInterface							*GetMutex()  {return m_pIOMutex;}
	TickCountInterface						*GetTickCountInterface() {return m_pTickCount;}

    void                                    portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const;

	bool                                    m_bLinked;
	int                                     m_nPosition;
    double                                  m_fLastTemp;
    CArduinoFocus                           m_ArduinoFocus;
    bool                                    mUiEnabled;
    bool                                    m_bContinous;
    bool                                    m_bReversed;
};


#endif //__X2Focuser_H_
