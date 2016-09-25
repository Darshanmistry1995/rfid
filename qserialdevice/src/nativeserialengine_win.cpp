/*
* This file is part of QSerialDevice, an open-source cross-platform library
* Copyright (C) 2009  Shienkov Denis
*
* This library is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
* Contact Shienkov Denis:
*          e-mail: <scapig2@yandex.ru>
*             ICQ: 321789831
*/

#include <qt_windows.h>

#include <QStringList>
#include <QAbstractEventDispatcher>

#include "nativeserialengine.h"


#define NATIVESERIALENGINE_WIN_DEBUG

#ifdef NATIVESERIALENGINE_WIN_DEBUG
#include <QDebug>
#include <QString>
#include <ctype.h>
#endif


NativeSerialEnginePrivate::NativeSerialEnginePrivate()
	: hd(INVALID_HANDLE_VALUE), readNotifier(0), writeNotifier(0)
{
}

bool NativeSerialEnginePrivate::isValid() const
{
	return hd != INVALID_HANDLE_VALUE;
}

QStringList NativeSerialEnginePrivate::nativeSerialDevicesAvailable() const
{
	QStringList devices; devices.clear();
	DWORD i = 0;
	DWORD keyType = 0;							//тип ключа
	const int MAX_KEY_LEN = 256;				//максимальная длина имени или знчения ключа
	char keyName[MAX_KEY_LEN];					//имя ключа
	char keyValue[MAX_KEY_LEN];					//значение ключа
	DWORD keyNameLength = sizeof(keyName);		//размер имени ключа в байтах
	DWORD keyValueLength = sizeof(keyValue);	//размер значения ключа в байтах
	HKEY hKey = 0;								//дескриптор ключа

	LONG rc = ::RegOpenKeyExA(HKEY_LOCAL_MACHINE,"Hardware\\Devicemap\\Serialcomm", 0, KEY_READ, &hKey);
	if (rc != ERROR_SUCCESS) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSerialDevicesAvailable->function RegOpenKeyExA returned " << rc << ". Error!";
#endif
		return devices;
	}

	while (::RegEnumValueA(hKey, i++, keyName, &keyNameLength, 0, &keyType, (BYTE *)keyValue, &keyValueLength) == ERROR_SUCCESS) {
		if (keyType == REG_SZ) devices << keyValue;
		keyNameLength = sizeof(keyName);
		keyValueLength = sizeof(keyValue);
	}

	rc = ::RegCloseKey(hKey);
	if (rc != ERROR_SUCCESS)
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSerialDevicesAvailable->function RegCloseKey returned " << rc << ". Error!";
#endif
	return devices;
}

bool NativeSerialEnginePrivate::nativeSetBaudRate(AbstractSerial::BaudRate baudRate)
{
	if (hd != INVALID_HANDLE_VALUE) {
		switch (baudRate) {
			case AbstractSerial::BaudRate50:
			case AbstractSerial::BaudRate75:
			case AbstractSerial::BaudRate134:
			case AbstractSerial::BaudRate150:
			case AbstractSerial::BaudRate200:
			case AbstractSerial::BaudRate1800:
			case AbstractSerial::BaudRate76800:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetBaudRate->Windows does not support" << baudRate << "baud operation. Error!";
#endif
				return false;
			case AbstractSerial::BaudRate110: cc.dcb.BaudRate = CBR_110; break;
			case AbstractSerial::BaudRate300: cc.dcb.BaudRate = CBR_300; break;
			case AbstractSerial::BaudRate600: cc.dcb.BaudRate = CBR_600; break;
			case AbstractSerial::BaudRate1200: cc.dcb.BaudRate = CBR_1200; break;
			case AbstractSerial::BaudRate2400: cc.dcb.BaudRate = CBR_2400; break;
			case AbstractSerial::BaudRate4800: cc.dcb.BaudRate = CBR_4800; break;
			case AbstractSerial::BaudRate9600: cc.dcb.BaudRate = CBR_9600; break;
			case AbstractSerial::BaudRate14400: cc.dcb.BaudRate = CBR_14400; break;
			case AbstractSerial::BaudRate19200: cc.dcb.BaudRate = CBR_19200; break;
			case AbstractSerial::BaudRate38400: cc.dcb.BaudRate = CBR_38400; break;
			case AbstractSerial::BaudRate56000: cc.dcb.BaudRate = CBR_56000; break;
			case AbstractSerial::BaudRate57600: cc.dcb.BaudRate = CBR_57600; break;
			case AbstractSerial::BaudRate115200: cc.dcb.BaudRate = CBR_115200; break;
			case AbstractSerial::BaudRate128000: cc.dcb.BaudRate = CBR_128000; break;
			case AbstractSerial::BaudRate256000: cc.dcb.BaudRate = CBR_256000; break;
			default:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetBaudRate-> " << baudRate << "not supported by the class NativeSerialEnginePrivate,";
    qDebug() << "see enum AbstractSerial::BaudRate. Error!";
#endif
				return false;
		}//switch
		if (::SetCommConfig(hd, &cc, sizeof(COMMCONFIG)) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetBaudRate->function SetCommConfig returned 0. Error!";
#endif
			return false;
		}
		m_baudRate = baudRate;
		return true;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetBaudRate->Device is not open. Error!");
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeSetDataBits(AbstractSerial::DataBits dataBits)
{
	if (hd != INVALID_HANDLE_VALUE) {
		if ((dataBits == AbstractSerial::DataBits5) && (m_stopBits == AbstractSerial::StopBits2)) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetDataBits->5 data bits cannot be used with 2 stop bits. Error!");
#endif
			return false;
		}
		if ((dataBits == AbstractSerial::DataBits6) && (m_stopBits == AbstractSerial::StopBits1_5)) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetDataBits->6 data bits cannot be used with 1.5 stop bits. Error!");
#endif
			return false;
		}
		if ((dataBits == AbstractSerial::DataBits7) && (m_stopBits == AbstractSerial::StopBits1_5)) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetDataBits->7 data bits cannot be used with 1.5 stop bits. Error!");
#endif
			return false;
		}
		if ((dataBits == AbstractSerial::DataBits8) && (m_stopBits == AbstractSerial::StopBits1_5)) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetDataBits->8 data bits cannot be used with 1.5 stop bits. Error!");
#endif
			return false;
		}
		switch(dataBits) {
			case AbstractSerial::DataBits5: cc.dcb.ByteSize = 5; break;
			case AbstractSerial::DataBits6: cc.dcb.ByteSize = 6; break;
			case AbstractSerial::DataBits7: cc.dcb.ByteSize = 7; break;
			case AbstractSerial::DataBits8: cc.dcb.ByteSize = 8; break;
			default:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetDataBits-> " << dataBits << "is not supported by the class NativeSerialEnginePrivate,";
    qDebug() << "see enum AbstractSerial::DataBits. Error!";
#endif
				return false;
		}//switch dataBits
			if (::SetCommConfig(hd, &cc, sizeof(COMMCONFIG)) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetDataBits->function SetCommConfig returned 0. Error!";
#endif
			return false;
		}
		m_dataBits = dataBits;
		return true;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetDataBits->Device is not open. Error!");
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeSetParity(AbstractSerial::Parity parity)
{
	if (hd != INVALID_HANDLE_VALUE) {
		switch (parity) {
			case AbstractSerial::ParityNone: cc.dcb.Parity = NOPARITY; cc.dcb.fParity = false; break;
			case AbstractSerial::ParitySpace: cc.dcb.Parity = SPACEPARITY; cc.dcb.fParity = true; break;
			case AbstractSerial::ParityMark: cc.dcb.Parity = MARKPARITY; cc.dcb.fParity = true; break;
			case AbstractSerial::ParityEven: cc.dcb.Parity = EVENPARITY; cc.dcb.fParity = true; break;
			case AbstractSerial::ParityOdd: cc.dcb.Parity = ODDPARITY; cc.dcb.fParity = true; break;
			default:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetParity-> " << parity << "is not supported by the class NativeSerialEnginePrivate,";
    qDebug() << "see enum AbstractSerial::Parity. Error!";
#endif
				return false;
		}//switch parity
		if (::SetCommConfig(hd, &cc, sizeof(COMMCONFIG)) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetParity->function SetCommConfig returned 0. Error!";
#endif
			return false;
		}
		m_parity = parity;
		return true;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetParity->Device is not open. Error!");
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeSetStopBits(AbstractSerial::StopBits stopBits)
{
	if (hd != INVALID_HANDLE_VALUE) {
		if ( (m_dataBits == AbstractSerial::DataBits5) && (m_stopBits == AbstractSerial::StopBits2) ) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetStopBits->5 data bits cannot be used with 2 stop bits. Error!");
#endif
			return false;
		}
		if ((m_dataBits == AbstractSerial::DataBits6) && (m_stopBits == AbstractSerial::StopBits1_5)) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetStopBits->6 data bits cannot be used with 1.5 stop bits. Error!");
#endif
			return false;
		}
		if ((m_dataBits == AbstractSerial::DataBits7) && (m_stopBits == AbstractSerial::StopBits1_5)) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetStopBits->7 data bits cannot be used with 1.5 stop bits. Error!");
#endif
			return false;
		}
		if ((m_dataBits == AbstractSerial::DataBits8) && (m_stopBits == AbstractSerial::StopBits1_5)) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetStopBits->8 data bits cannot be used with 1.5 stop bits. Error!");
#endif
			return false;
		}
		switch (stopBits) {
			case AbstractSerial::StopBits1: cc.dcb.StopBits = ONESTOPBIT; break;
			case AbstractSerial::StopBits1_5: cc.dcb.StopBits = ONE5STOPBITS; break;
			case AbstractSerial::StopBits2: cc.dcb.StopBits = TWOSTOPBITS; break;
			default:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetStopBits-> " << stopBits << "is not supported by the class NativeSerialEnginePrivate,";
    qDebug() << "see enum AbstractSerial::StopBits. Error!";
#endif
				return false;
		}//switch stopBits
		if (::SetCommConfig(hd, &cc, sizeof(COMMCONFIG)) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetStopBits->function SetCommConfig returned 0. Error!";
#endif
			return false;
		}
		m_stopBits=stopBits;
		return true;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetStopBits->Device is not open. Error!");
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeSetFlowControl(AbstractSerial::Flow flow)
{
	if (hd != INVALID_HANDLE_VALUE) {
		switch(flow) {
			case AbstractSerial::FlowControlOff:
				cc.dcb.fOutxCtsFlow = false; cc.dcb.fRtsControl = RTS_CONTROL_DISABLE;
				cc.dcb.fInX = false; cc.dcb.fOutX = false; break;
			case AbstractSerial::FlowControlXonXoff:
				cc.dcb.fOutxCtsFlow = false; cc.dcb.fRtsControl = RTS_CONTROL_DISABLE;
				cc.dcb.fInX = true; cc.dcb.fOutX = true; break;
			case AbstractSerial::FlowControlHardware:
				cc.dcb.fOutxCtsFlow = true; cc.dcb.fRtsControl = RTS_CONTROL_HANDSHAKE;
				cc.dcb.fInX = false; cc.dcb.fOutX = false; break;
			default:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetFlowControl-> " << flow << "is not supported by the class NativeSerialEnginePrivate,";
    qDebug() << "see enum AbstractSerial::Flow. Error!";
#endif
				return false;
		}//switch flow
		if (::SetCommConfig(hd, &cc, sizeof(COMMCONFIG)) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetFlowControl->function SetCommConfig returned 0. Error!";
#endif
			return false;
		}
		m_flow = flow;
		return true;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetFlowControl->Device is not open. Error!");
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeSetCharIntervalTimeout(int msecs)
{
	if (hd != INVALID_HANDLE_VALUE) {
		/* тут нужно задать настройки для АСИНХРОННОГО режима! нужно проверить!
		(here you need to specify the settings for the asynchronous mode! to check!) */
		ct.ReadIntervalTimeout = MAXDWORD;
		ct.ReadTotalTimeoutMultiplier = 0;
		ct.ReadTotalTimeoutConstant = 0;
		ct.WriteTotalTimeoutMultiplier = 0;
		ct.WriteTotalTimeoutConstant = 0;
		if (::SetCommTimeouts(hd, &ct) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSetCharIntervalTimeout->function SetCommTimeouts returned  0. Error!";
#endif
			return false;
		}
		m_charIntervalTimeout = msecs;
		return true;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetCharIntervalTimeout->Device is not open. Error!");
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeOpen(QIODevice::OpenMode mode)
{
	if (hd == INVALID_HANDLE_VALUE) {
		DWORD access = 0;
		DWORD sharing = 0;
		DWORD confSize = sizeof(COMMCONFIG);
		::ZeroMemory(&cc, confSize);
		::ZeroMemory(&oldcc, confSize);
		cc.dwSize = confSize;
		oldcc.dwSize = confSize;

		switch (mode & (~QIODevice::Unbuffered)) {
			case QIODevice::ReadOnly:
				access = GENERIC_READ; sharing = FILE_SHARE_READ;
				break;
			case QIODevice::WriteOnly:
				access = GENERIC_WRITE; sharing = FILE_SHARE_WRITE;
				break;
			case QIODevice::ReadWrite:
				access = GENERIC_READ|GENERIC_WRITE; sharing = FILE_SHARE_READ|FILE_SHARE_WRITE;
				break;
			/*
			case QIODevice::NotOpen:
			case QIODevice::Append:
			case QIODevice::Truncate:
			case QIODevice::Text:
			case QIODevice::Unbuffered:
			*/
			default:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeOpen->mode = " << mode << " undefined. Error!";
#endif
				return false;
		}

		//открываем последовательное устройство (opened serial device)
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeOpen->Trying to open device: " << m_deviceName;
#endif
		hd = ::CreateFileA(m_deviceName.toAscii(),
						access, sharing,
						0, OPEN_EXISTING,
						FILE_FLAG_NO_BUFFERING|FILE_FLAG_OVERLAPPED,
						0);
		if (hd == INVALID_HANDLE_VALUE) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeOpen->function CreateFileA returned  " << hd << ". Error!";
#endif
			return false;
		}

		if (!nativeReset()) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeOpen->function nativeReset returned false. Error!";
#endif
			return false;
		}

		//сохраняем текущие установки устройства в oldtio (saved current parameters device)
		if (::GetCommConfig(hd, &oldcc, &confSize) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeOpen->function GetCommConfig returned 0. Error!";
#endif
			return false;
		}

		//выставляем флаг того, что успешно сохранили старые настройки устройства до его конфигурирования
		m_oldSettingsIsSaved = true;// set flag "altered parameters is saved"

		//приступаем к конфигурированию открытого устройства (get configure)
		memcpy(&cc, &oldcc, sizeof(COMMCONFIG));

		cc.dcb.fBinary = true;
		cc.dcb.fInX = false;
		cc.dcb.fOutX = false;
		cc.dcb.fAbortOnError = false;
		cc.dcb.fNull = false;

		if ( !(nativeSetBaudRate(m_baudRate) &&
			nativeSetDataBits(m_dataBits) &&
			nativeSetParity(m_parity) &&
			nativeSetStopBits(m_stopBits) &&
			nativeSetFlowControl(m_flow) &&
			nativeSetCharIntervalTimeout(m_charIntervalTimeout)) ) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeOpen->Set default parameters. Error!";
#endif
			return false;
		}//if set parameters

		if (::SetCommConfig(hd, &cc, sizeof(COMMCONFIG)) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeOpen->function SetCommConfig returned 0. Error!";
#endif
			return false;
		}////if set all param
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeOpen->Opened device succesfully. Ok!";
#endif
		return true;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeOpen-> The device is already open or other error";
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeClose()
{
	if ( hd != INVALID_HANDLE_VALUE ) {
		bool okClosed = true;
		//восстанавливаем старые сохраненные параметры устройства (repair altered parameters device)
		if (m_oldSettingsIsSaved)
			if (::SetCommConfig(hd, &oldcc, sizeof(COMMCONFIG)) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeClose->function SetCommConfig returned 0. Error!");
#endif
				okClosed = false;
			}
		//закрываем устройство (closed device)
		if (::CloseHandle(hd) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeClose->function CloseHandle returned 0. Error!");
#endif
			okClosed = false;
		}
		hd = INVALID_HANDLE_VALUE;
		return okClosed;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeClose->INVALID_HANDLE_VALUE or device is not open. Error!");
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeFlush()
{
	if (hd != INVALID_HANDLE_VALUE) {
		bool ret = ::FlushFileBuffers(hd);
		if (!ret) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeFlush->function FlushFileBuffers returned false. Error!");
#endif
		}
		return ret;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeFlush->Device is not open. Error!");
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeReset()
{
	if (hd != INVALID_HANDLE_VALUE) {
		bool ret = true;

		// cброс буферов порта. (reset device buffers)
		if (::PurgeComm(hd,PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeReset->function PurgeComm returned 0. Error!");
#endif
			ret = false;
		}
		// cброс регистров порта. (reset device registers)
		if (::ClearCommBreak(hd) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeReset->function ClearCommBreak returned 0. Error!");
#endif
			ret = false;
		}

		return ret;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeReset->Device is not open. Error!");
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeSetDtr(bool set)
{
	if (hd != INVALID_HANDLE_VALUE) {
		if (::EscapeCommFunction(hd, (set) ? SETDTR : CLRDTR) == -1) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetDtr->function EscapeCommFunction returned -1. Error!");
#endif
			return false;
		}
		//setStatus(AbstractSerial::ENone);
		return true;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetDtr->Device is not open. Error!");
#endif
	return false;
}

bool NativeSerialEnginePrivate::nativeSetRts(bool set)
{
	if (hd != INVALID_HANDLE_VALUE) {
		if (::EscapeCommFunction(hd, (set) ? SETRTS : CLRRTS) == -1) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetRts->function EscapeCommFunction returned -1. Error!");
#endif
			return false;
		}
		return true;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSetRts->Device is not open. Error!");
#endif
	return false;
}

ulong NativeSerialEnginePrivate::nativeLineStatus()
{
	if (hd != INVALID_HANDLE_VALUE) {
		ulong temp = 0, status = 0;
		if (::GetCommModemStatus(hd, &temp) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeLineStatus->function GetCommModemStatus returned 0. Error!");
#endif
			return status;
		}
		if (temp&MS_CTS_ON) status|=LS_CTS;
		if (temp&MS_DSR_ON) status|=LS_DSR;
		if (temp&MS_RING_ON) status|=LS_RI;
		if (temp&MS_RLSD_ON) status|=LS_DCD;
		return status;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeLineStatus->Device is not open. Error!");
#endif
	return 0;
}

qint64 NativeSerialEnginePrivate::nativeBytesAvailable(bool wait)
{
	if (hd != INVALID_HANDLE_VALUE) {
		DWORD err = 0;
		DWORD bav = 0;
		COMSTAT cs; ::ZeroMemory(&cs, sizeof(cs));
		for(;;) {
			if (ClearCommError(hd, &err, &cs) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeBytesAvailable->function ClearCommError returned 0. Error!");
#endif
				return (qint64)-1;
			}
			if (err != 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeBytesAvailable->function ClearCommError(err = " << err << ")";
#endif
				return (qint64)-1;
			}

			bav = cs.cbInQue;
			if (!wait) 
				break;
			if (bav > 0)
				break;
			if (!nativeSelect(m_charIntervalTimeout, true)) 
				break;
		}
		return (qint64)bav;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeBytesAvailable->Device is not open. Error!");
#endif
	return 0;
}

qint64 NativeSerialEnginePrivate::nativeWrite(const char *data, qint64 len)
{
	if (hd != INVALID_HANDLE_VALUE) {

		if (len <= 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeWrite-> Input parameter len = " << len << ". Error!";
#endif
			return (qint64)-1;
		}
		OVERLAPPED ovl; ::ZeroMemory(&ovl, sizeof(ovl));
		static const int W_CHUNK_SIZE = 256;
		qint64 bytesToSend = 0;
		DWORD bytesWritten = 0;
		qint64 ret = 0;

		for (;;) {
			bytesToSend = qMin<qint64>(W_CHUNK_SIZE, len - ret);

			bool fSucess = false;
			if (!::WriteFile(hd, (void*)(data+ret), (DWORD)bytesToSend, &bytesWritten, &ovl)) {
				DWORD rc = ::GetLastError();
				if (rc == ERROR_IO_PENDING ) {
					if (!::GetOverlappedResult(hd, &ovl, &bytesWritten, true)) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeWrite->function GetOverlappedResult returned false. Error!");
#endif
						ret = -1; break;
					}
					else {
						fSucess = true;
					}
				}
				else {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeWrite->function GetLastError returned " << rc << ". Error!";
#endif
					ret = -1; break;
				}
			}
			else {
				fSucess = true;
			}

			if (fSucess) {
				if (bytesWritten == bytesToSend)
					ret += (quint64) bytesWritten;
				else {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeWrite->bytesWritten != bytesToSend. Error!";
    qDebug() << " - states for writting bytesToSend is "  << bytesToSend << " bytes";
    qDebug() << " - actually write bytesWritten is "  << bytesWritten << " bytes";
#endif
					ret = -1; break;
				}
			}
			if (ret == len) break;
		}
		return ret;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeWrite->Device is not open. Error!");
#endif
	return (qint64)-1;
}

qint64 NativeSerialEnginePrivate::nativeRead(char *data, qint64 len)
{
	if (hd != INVALID_HANDLE_VALUE) {

		if (len <= 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeRead-> Input parameter len = " << len << ". Error!";
#endif
			return (qint64)-1;
		}
		OVERLAPPED ovl; ::ZeroMemory(&ovl, sizeof(ovl));
		qint64 bytesToRead = 0;
		DWORD bytesReaded = 0;
		qint64 bav = 0;
		qint64 ret = 0;

		for (;;) {
			bav = nativeBytesAvailable(true);
			if (bav < 0) {
				ret = -1; break;
			}

			if (bav == 0) 
				break;

			bytesToRead = qMin<qint64>(len - ret, bav);

			bool fSucess = false;
			if ( !(::ReadFile(hd, (void*)(data+ret), bytesToRead, &bytesReaded, &ovl)) ) {
				DWORD rc = ::GetLastError();
				if (rc == ERROR_IO_PENDING ) {
					if (!::GetOverlappedResult(hd, &ovl, &bytesReaded, true)) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeRead->function GetOverlappedResult returned false. Error!");
#endif
						ret = -1; break;
					}
					else {
						fSucess = true;
					}
				} // ERROR_IO_PENDING
				else {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeRead->function GetLastError returned " << rc << ". Error!";
#endif
					ret = -1; break;
				}
			}
			else {
				fSucess = true;
			}

			if (fSucess) {
				if (bytesReaded == bytesToRead)
					ret += qint64(bytesReaded);
				else {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeRead->bytesReaded != bytesToRead. Error!";
    qDebug() << " - states for reading is"  << bytesToRead << "bytes";
    qDebug() << " - actually read is"  << bytesReaded << "bytes";
#endif
					ret = -1; break;
				}
			}
			if (ret == len) break;
		}
		return ret;
	}
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeRead->Device is not open. Error!");
#endif
	return (qint64)-1;
}

int NativeSerialEnginePrivate::nativeSelect(int timeout, bool selectForRead) const
{
	int ret = -1;
	DWORD eventMask = 0;

	//создаем событие (create event)
	OVERLAPPED ovl; ::ZeroMemory(&ovl, sizeof(ovl));
	ovl.hEvent = ::CreateEvent(0, false, false, 0);

	if (ovl.hEvent == INVALID_HANDLE_VALUE) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect1->function CreateEvent returned " << ovl.hEvent << ". Error!";
#endif
		return ret;
	}

	if(::GetCommMask(hd, &eventMask) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect1->function GetCommMask(eventMask = " << eventMask << ") returned 0. Error!";
#endif
		return ret;
	}

	if (selectForRead)
		if ( (eventMask & EV_RXCHAR) == 0 ) 
			eventMask |= EV_RXCHAR;
	else
		if ( (eventMask & EV_TXEMPTY) == 0 ) 
			eventMask |= EV_TXEMPTY;

		//установили маску
	if(::SetCommMask(hd, eventMask) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect1->function SetCommMask(eventMask = " << eventMask << ") returned 0. Error!";
#endif
		return ret;
	}
	bool fSucess = false;
	if (::WaitCommEvent(hd, &eventMask, &ovl) == 0) {
		DWORD rc = ::GetLastError();
		if (rc == ERROR_IO_PENDING) {
			rc = ::WaitForSingleObject(ovl.hEvent, timeout < 0 ? 0 : timeout);
			switch (rc) {
				case WAIT_OBJECT_0:
					fSucess = true; break;
				case WAIT_TIMEOUT:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect1->function WaitForSingleObject returned WAIT_TIMEOUT : " << rc << ". Warning!";
#endif
					ret = 0; break;
				default:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect1->function WaitForSingleObject returned undefined event code " << rc << ". Error!";
#endif
					;
			}
		}
		else {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect1->function GetLastError returned " << rc << ". Error!";
#endif
		}
	}
	else { 
		fSucess = true;
	}

	if (fSucess) {
		fSucess = false;
		if (selectForRead)
			if ( (eventMask & EV_RXCHAR) == EV_RXCHAR ) {
				ret = 1;
				fSucess = true;
			}
		else
			if ( (eventMask & EV_TXEMPTY) == EV_TXEMPTY ) {
				ret = 1;
				fSucess = true;
			}

		if (!fSucess) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect1->function WaitCommEvent returned undefined mask event " << eventMask <<". Error!";
#endif
		}
	}

	if (::CloseHandle(ovl.hEvent) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSelect1->function CloseHandle returned 0. Error!");
#endif
		ret = -1;
	}
	return ret;
}

int NativeSerialEnginePrivate::nativeSelect(int timeout,
										bool checkRead, bool checkWrite,
										bool *selectForRead, bool *selectForWrite) const
{
	int ret = -1;
	DWORD eventMask = 0;

	//создаем событие (create event)
	OVERLAPPED ovl; ::ZeroMemory(&ovl, sizeof(ovl));
	ovl.hEvent = ::CreateEvent(0, false, false, 0);

	if (ovl.hEvent == INVALID_HANDLE_VALUE) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect2->function CreateEvent returned " << ovl.hEvent << ". Error!";
#endif
		return ret;
	}

	if(::GetCommMask(hd, &eventMask) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect2->function GetCommMask(eventMask = " << eventMask << ") returned 0. Error!";
#endif
		return ret;
	}

	if (selectForRead)
		if ( (eventMask & EV_RXCHAR) == 0 ) 
			eventMask |= EV_RXCHAR;
	else
		if ( (eventMask & EV_TXEMPTY) == 0 ) 
			eventMask |= EV_TXEMPTY;

	//установили маску
	if(::SetCommMask(hd, eventMask) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect2->function SetCommMask(eventMask = " << eventMask << ") returned 0. Error!";
#endif
		return ret;
	}
	bool fSucess = false;
	if (::WaitCommEvent(hd, &eventMask, &ovl) == 0) {
		DWORD rc = ::GetLastError();
		if (rc == ERROR_IO_PENDING) {
			rc = ::WaitForSingleObject(ovl.hEvent, timeout < 0 ? 0 : timeout);
			switch (rc) {
				case WAIT_OBJECT_0:
					fSucess = true; break;
				case WAIT_TIMEOUT:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect2->function WaitForSingleObject returned WAIT_TIMEOUT : " << rc << ". Warning!";
#endif
					ret = 0; break;
				default:
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect2->function WaitForSingleObject returned undefined event code " << rc << ". Error!";
#endif
					;
			}
		}
		else {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect2->function GetLastError returned " << rc << ". Error!";
#endif
		}
	}
	else {
		fSucess = true;
	}

	if (fSucess) {
		fSucess = false;

		*selectForRead = (eventMask & EV_RXCHAR) ? true : false;
		*selectForWrite = (eventMask & EV_TXEMPTY) ? true : false;

		if (*selectForRead || *selectForWrite) 
			ret = 1;
		else {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug() << "Windows: NativeSerialEnginePrivate::nativeSelect2->function WaitCommEvent returned undefined mask event " << eventMask <<". Error!";
#endif
		}
	}

	if (::CloseHandle(ovl.hEvent) == 0) {
#if defined (NATIVESERIALENGINE_WIN_DEBUG)
    qDebug("Windows: NativeSerialEnginePrivate::nativeSelect2->function CloseHandle returned 0. Error!");
#endif
		ret = -1;
	}
	return ret;
}

//------------------------------------------------------------------------------------------//

bool NativeSerialEngine::isReadNotificationEnabled() const
{
	Q_D(const NativeSerialEngine);
	return d->readNotifier && d->readNotifier->isEnabled();
}

void NativeSerialEngine::setReadNotificationEnabled(bool enable)
{
	Q_D(NativeSerialEngine);
	if (d->readNotifier) {
		d->readNotifier->setEnabled(enable);
	} else if (enable && QAbstractEventDispatcher::instance(thread())) {
		d->readNotifier = new WinSerialNotifier(d->hd, WinSerialNotifier::Read, this);
		QObject::connect(d->readNotifier, SIGNAL(activated(HANDLE)),
							this, SIGNAL(readNotification()));
		d->readNotifier->setEnabled(true);
	}
}

bool NativeSerialEngine::isWriteNotificationEnabled() const
{
	Q_D(const NativeSerialEngine);
	return d->writeNotifier && d->writeNotifier->isEnabled();
}

void NativeSerialEngine::setWriteNotificationEnabled(bool enable)
{
	Q_D(NativeSerialEngine);
	if (d->writeNotifier) {
		d->writeNotifier->setEnabled(enable);
	} else if (enable && QAbstractEventDispatcher::instance(thread())) {
		d->writeNotifier = new WinSerialNotifier(d->hd, WinSerialNotifier::Write, this);
		QObject::connect(d->writeNotifier, SIGNAL(activated(HANDLE)),
							this, SIGNAL(writeNotification()));

		d->writeNotifier->setEnabled(true);
	}
}
