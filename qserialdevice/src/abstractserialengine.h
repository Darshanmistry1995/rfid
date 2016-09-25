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

#ifndef ABSTRACTSERIALENGINE_H
#define ABSTRACTSERIALENGINE_H

#include <QObject>

#include "abstractserial.h"

class AbstractSerialEnginePrivate;
class AbstractSerialEngine : public QObject
{
	Q_OBJECT
public:
	AbstractSerialEngine(QObject *parent = 0);
	virtual ~AbstractSerialEngine();

	static AbstractSerialEngine *createSerialEngine(QObject *parent);

	//my impl (device parameters and controls)
	void setDeviceName(const QString &deviceName);
	virtual QStringList serialDevicesAvailable() const = 0;
	virtual bool setBaudRate(AbstractSerial::BaudRate baudRate) = 0;
	virtual bool setDataBits(AbstractSerial::DataBits dataBits) = 0;
	virtual bool setParity(AbstractSerial::Parity parity) = 0;
	virtual bool setStopBits(AbstractSerial::StopBits stopBits) = 0;
	virtual bool setFlowControl(AbstractSerial::Flow flow) = 0;
	virtual bool setCharIntervalTimeout(int msecs) = 0;

	virtual bool open(QIODevice::OpenMode mode) = 0;
	virtual void close() = 0;
	virtual bool flush() = 0;
	virtual bool reset() = 0;

	//управление линиями DTR, RTS (control DTR and RTS lines)
	virtual bool setDtr(bool set) = 0;
	virtual bool setRts(bool set) = 0;
	//получение статуса линий DSR, RTS и т.п. (get state DSR and RTS lines)
	virtual ulong lineStatus() = 0;

	virtual qint64 bytesAvailable(bool wait) = 0;

	virtual qint64 read(char *data, qint64 maxlen) = 0;
	virtual qint64 write(const char *data, qint64 len) = 0;

	virtual bool waitForRead(int msecs = 3000, bool *timedOut = 0) const = 0;
	virtual bool waitForWrite(int msecs = 3000, bool *timedOut = 0) const = 0;
	virtual bool waitForReadOrWrite(bool *readyToRead, bool *readyToWrite,
				bool checkRead, bool checkWrite,
				int msecs = 3000, bool *timedOut = 0) const = 0;

	//my impl
	QString deviceName() const;
	AbstractSerial::BaudRate baudRate() const;
	AbstractSerial::DataBits dataBits() const;
	AbstractSerial::Parity parity() const;
	AbstractSerial::StopBits stopBits() const;
	AbstractSerial::Flow flow() const;
	int charIntervalTimeout() const;
	//
	AbstractSerial::Status status() const;

	virtual bool isReadNotificationEnabled() const = 0;
	virtual void setReadNotificationEnabled(bool enable) = 0;
	virtual bool isWriteNotificationEnabled() const = 0;
	virtual void setWriteNotificationEnabled(bool enable) = 0;

Q_SIGNALS:
	void readNotification();
	void writeNotification();

protected:
	AbstractSerialEnginePrivate * const d_ptr;
	AbstractSerialEngine(AbstractSerialEnginePrivate &dd, QObject *parent);
private:
	Q_DECLARE_PRIVATE(AbstractSerialEngine)
	Q_DISABLE_COPY(AbstractSerialEngine)
};

class AbstractSerialEnginePrivate
{
public:
	AbstractSerialEnginePrivate();

	QString m_deviceName;
	AbstractSerial::BaudRate m_baudRate;
	AbstractSerial::DataBits m_dataBits;
	AbstractSerial::Parity m_parity;
	AbstractSerial::StopBits m_stopBits;
	AbstractSerial::Flow m_flow;

	int m_charIntervalTimeout;

	AbstractSerial::Status m_status;

	bool m_oldSettingsIsSaved;
};

#endif // ABSTRACTSERIALENGINE_H
