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

#ifndef NATIVESERIALENGINE_H
#define NATIVESERIALENGINE_H

#include "abstractserialengine.h"

class NativeSerialEnginePrivate;
class NativeSerialEngine : public AbstractSerialEngine
{
	Q_OBJECT
public:
	NativeSerialEngine(QObject *parent = 0);
	virtual ~NativeSerialEngine();

	bool isValid() const;

	//my impl (device parameters and controls)
	QStringList serialDevicesAvailable() const;
	bool setBaudRate(AbstractSerial::BaudRate baudRate);
	bool setDataBits(AbstractSerial::DataBits dataBits);
	bool setParity(AbstractSerial::Parity parity);
	bool setStopBits(AbstractSerial::StopBits stopBits);
	bool setFlowControl(AbstractSerial::Flow flow);
	bool setCharIntervalTimeout(int msecs);

	bool open(QIODevice::OpenMode mode);
	void close();
	bool flush();
	bool reset();

	bool setDtr(bool set);
	bool setRts(bool set);

	ulong lineStatus();

	qint64 bytesAvailable(bool wait);

	qint64 read(char *data, qint64 maxlen);
	qint64 write(const char *data, qint64 len);

	bool waitForRead(int msecs = 3000, bool *timedOut = 0) const;
	bool waitForWrite(int msecs = 3000, bool *timedOut = 0) const;
	bool waitForReadOrWrite(bool *readyToRead, bool *readyToWrite,
							bool checkRead, bool checkWrite,
							int msecs = 3000, bool *timedOut = 0) const;

	bool isReadNotificationEnabled() const;
	void setReadNotificationEnabled(bool enable);
	bool isWriteNotificationEnabled() const;
	void setWriteNotificationEnabled(bool enable);

private:
	Q_DECLARE_PRIVATE(NativeSerialEngine)
	Q_DISABLE_COPY(NativeSerialEngine)
};

#ifdef Q_OS_WIN
#include "winserialnotifier.h"
#else
#include <QSocketNotifier>
#include <termios.h>
#endif

class NativeSerialEnginePrivate : public AbstractSerialEnginePrivate
{
public:
	NativeSerialEnginePrivate();

#ifdef Q_OS_WIN
	HANDLE hd;
	COMMCONFIG cc, oldcc;
	COMMTIMEOUTS ct;
	WinSerialNotifier *readNotifier, *writeNotifier;
#else
	int fd;
	struct termios tio;
	struct termios oldtio;
	QSocketNotifier *readNotifier, *writeNotifier;
#endif

	bool isValid() const;

	QStringList nativeSerialDevicesAvailable() const;
	bool nativeSetBaudRate(AbstractSerial::BaudRate baudRate);
	bool nativeSetDataBits(AbstractSerial::DataBits dataBits);
	bool nativeSetParity(AbstractSerial::Parity parity);
	bool nativeSetStopBits(AbstractSerial::StopBits stopBits);
	bool nativeSetFlowControl(AbstractSerial::Flow flow);
	bool nativeSetCharIntervalTimeout(int msecs);
	//
	bool nativeOpen(QIODevice::OpenMode mode);
	bool nativeClose();
	bool nativeFlush();
	bool nativeReset();

	bool nativeSetDtr(bool set);
	bool nativeSetRts(bool set);

	ulong nativeLineStatus();

	qint64 nativeBytesAvailable(bool wait);

	qint64 nativeRead(char *data, qint64 len);
	qint64 nativeWrite(const char *data, qint64 len);
	int nativeSelect(int timeout, bool selectForRead) const;
	int nativeSelect(int timeout, bool checkRead, bool checkWrite,
				bool *selectForRead, bool *selectForWrite) const;
};

#endif // NativeSerialEngine_H
