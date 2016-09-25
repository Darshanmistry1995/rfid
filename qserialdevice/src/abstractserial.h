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

#ifndef ABSTRACTSERIAL_H
#define ABSTRACTSERIAL_H

#include <QIODevice>
#include <QMap>
#include <QDateTime>

#include "datatypes.h"


class AbstractSerialPrivate;
class AbstractSerial : public QIODevice
{
	Q_OBJECT
signals:
	void signalStatus(const QString &status, QDateTime current);

public:
/*!
\enum BaudRate
\~russian В этом перечислении представлены все поддерживаемые классом \a AbstractSerial скорости работы.
*/
	enum BaudRate {
		BaudRate50		=	50,     /*!< \~russian скорость 50 бод \~english speed 50 bauds */
		BaudRate75		=	75,     /*!< \~russian скорость 75 бод \~english speed 75 bauds */
		BaudRate110		=	110,    /*!< \~russian скорость 110 бод \~english speed 110 bauds */
		BaudRate134		=	134,    /*!< \~russian скорость 134 бод \~english speed 134 bauds */
		BaudRate150		=	150,    /*!< \~russian скорость 150 бод \~english speed 150 bauds */
		BaudRate200		=	200,    /*!< \~russian скорость 200 бод \~english speed 200 bauds */
		BaudRate300		=	300,    /*!< \~russian скорость 300 бод \~english speed 300 bauds */
		BaudRate600		=	600,    /*!< \~russian скорость 600 бод \~english speed 600 bauds */
		BaudRate1200	=	1200,   /*!< \~russian скорость 1200 бод \~english speed 1200 bauds */
		BaudRate1800	=	1800,   /*!< \~russian скорость 1800 бод \~english speed 1800 bauds */
		BaudRate2400	=	2400,   /*!< \~russian скорость 2400 бод \~english speed 2400 bauds */
		BaudRate4800	=	4800,   /*!< \~russian скорость 4800 бод \~english speed 4800 bauds */
		BaudRate9600	=	9600,   /*!< \~russian скорость 9600 бод \~english speed 9600 bauds */
		BaudRate14400	=	14400,  /*!< \~russian скорость 14400 бод \~english speed 14400 bauds */
		BaudRate19200	=	19200,  /*!< \~russian скорость 19200 бод \~english speed 19200 bauds */
		BaudRate38400	=	38400,  /*!< \~russian скорость 38400 бод \~english speed 38400 bauds */
		BaudRate56000	=	56000,  /*!< \~russian скорость 56000 бод \~english speed 56000 bauds */
		BaudRate57600	=	57600,  /*!< \~russian скорость 57600 бод \~english speed 57600 bauds */
		BaudRate76800	=	76800,  /*!< \~russian скорость 76800 бод \~english speed 76800 bauds */
		BaudRate115200	=	115200, /*!< \~russian скорость 115200 бод \~english speed 115200 bauds */
		BaudRate128000	=	128000, /*!< \~russian скорость 128000 бод \~english speed 128000 bauds */
		BaudRate256000	=	256000  /*!< \~russian скорость 256000 бод \~english speed 256000 bauds */
	};
/*!
\enum DataBits
\~russian В этом перечислении представлены все поддерживаемые классом \a AbstractSerial количества бит данных.
*/
	enum DataBits {
		DataBits5		=	5,     /*!< \~russian 5 бит данных \~english 5 data bits */
		DataBits6		=	6,     /*!< \~russian 6 бит данных \~english 6 data bits */
		DataBits7		=	7,     /*!< \~russian 7 бит данных \~english 7 data bits */
		DataBits8		=	8      /*!< \~russian 8 бит данных \~english 8 data bits */
	};
/*!
\enum Parity
\~russian В этом перечислении представлены все поддерживаемые классом \a AbstractSerial типы контроля паритета.
*/
	enum Parity {
		ParityNone		=	0,     /*!< \~russian без контроля четности \~english parity off (none) */
		ParityOdd		=	1,     /*!< \~russian паритет "нечет" \~english odd parity */
		ParityEven		=	2,     /*!< \~russian паритет "чет" \~english even parity */
		ParityMark		=	3,     /*!< \~russian паритет "маркер" \~english mark parity */
		ParitySpace		=	4      /*!< \~russian паритет "пробел" \~english space parity */
	};
/*!
\enum StopBits
\~russian В этом перечислении представлены все поддерживаемые классом \a AbstractSerial количества стоп бит.
*/
	enum StopBits {
		StopBits1		=	1,     /*!< \~russian один стоп бит \~english one stop bit */
		StopBits1_5		=	15,    /*!< \~russian полтора стоп бита \~english 1.5 stop bit */
		StopBits2		=	2      /*!< \~russian два стоп бита \~english two stop bit */
	};
/*!
\enum Flow
\~russian В этом перечислении представлены все поддерживаемые классом \a AbstractSerial режимы управления потоком.
*/
	enum Flow {
		FlowControlOff		=	0, /*!< \~russian управление потоком отключено \~english flow control off */
		FlowControlHardware	=	1, /*!< \~russian управление потоком аппаратное \~english flow control hardware */
		FlowControlXonXoff	=	2  /*!< \~russian управление потоком Xon/Xoff \~english flow control Xon/Xoff */
	};
/*!
\enum Status
\~russian В этом перечислении представлены все поддерживаемые классом \a AbstractSerial статусы работы.
*/
	enum Status {
		/* group of "SUCESS STATES" */
		ENone 						= 0,   /*!< \~russian нет ошибок \~english  */
		ENoneOpen					= 1,   /*!< \~russian открыто без ошибок \~english  */
		ENoneClose					= 2,   /*!< \~russian закрыто без ошибок \~english  */
		ENoneSetBaudRate			= 3,   /*!< \~russian скорость установлена без ошибок \~english  */
		ENoneSetParity				= 4,   /*!< \~russian паритет установлен без ошибок \~english  */
		ENoneSetDataBits			= 5,   /*!< \~russian биты данных установлены без ошибок \~english  */
		ENoneSetStopBits			= 6,   /*!< \~russian стоп-биты установлены без ошибок \~english  */
		ENoneSetFlow				= 7,   /*!< \~russian управление потоком установлено без ошибок \~english  */
		ENoneSetCharTimeout			= 8,   /*!< \~russian интервал ожидания символа установлен без ошибок \~english  */
		// 9-14 reserved
		/* Groups of "ERROR STATES" */
		//group of "OPEN"
		EOpen						= 15,   /*!< \~russian ошибка при открытии \~english  */
		EDeviceIsNotOpen			= 16,   /*!< \~russian устройство еще не открыто \~english  */
		EOpenModeUnsupported		= 17,   /*!< \~russian режим открытия не поддерживается \~english  */
		EOpenModeUndefined			= 18,   /*!< \~russian режим открытия неопределен \~english  */
		EOpenInvalidFD				= 19,   /*!< \~russian недействительный дескриптор \~english  */
		EOpenOldSettingsNotSaved	= 20,   /*!< \~russian ошибка сохранения старых параметров при открытии \~english  */
		EOpenGetCurrentSettings		= 21,   /*!< \~russian ошибка получения текущих параметров при открытии \~english  */
		EOpenSetDefaultSettings		= 22,   /*!< \~russian ошибка установки параметров по умолчанию при открытии \~english  */
		EDeviceIsOpen				= 23,   /*!< \~russian устройство уже открыто \~english  */
		//group of "CLOSE"
		ECloseSetOldSettings		= 24,   /*!< \~russian ошибка сохранения старых параметров при закрытии \~english  */
		ECloseFD					= 25,   /*!< \~russian ошибка при закрытии дескриптора \~english  */
		EClose						= 26,   /*!< \~russian ошибка при закрытии \~english  */
		// 27-31 reserved
		//group of "SETTINGS"
		ESetBaudRate				= 32,   /*!< \~russian ошибка при установке скорости \~english  */
		ESetDataBits				= 33,   /*!< \~russian ошибка при установке кол-ва бит данных \~english  */
		ESetParity					= 34,   /*!< \~russian ошибка при установке паритета \~english  */
		ESetStopBits				= 35,   /*!< \~russian ошибка при установке кол-ва стоп бит \~english  */
		ESetFlowControl				= 36,   /*!< \~russian ошибка при установке управления потоком \~english  */
		ESetCharIntervalTimeout		= 37,   /*!< \~russian ошибка при установке интервала ожидания символа \~english  */
		// 38-39 reserved
		//group of "CONTROL"
		EBytesAvailable				= 40,   /*!< \~russian ошибка при получении байт готовых для чтения \~english  */
		ESetDtr						= 41,   /*!< \~russian ошибка при установке DTR \~english  */
		ESetRts						= 42,   /*!< \~russian ошибка при установке RTS \~english  */
		ELineStatus					= 43,   /*!< \~russian ошибка при получении статусов линий \~english  */
		EWaitReadyReadIO			= 44,   /*!< \~russian ошибка ввода/вывода при ожидании приема данных \~english  */
		EWaitReadyReadTimeout		= 45,   /*!< \~russian ошибка истечения времени при ожидании приема данных \~english  */
		EWaitReadyWriteIO			= 46,   /*!< \~russian ошибка ввода/вывода при ожидании передачи данных \~english  */
		EWaitReadyWriteTimeout		= 47,   /*!< \~russian ошибка истечения времени при ожидании передачи данных \~english  */
		EReadDataIO					= 48,   /*!< \~russian ошибка чтения данных \~english  */
		EWriteDataIO				= 49,   /*!< \~russian ошибка передачи данных \~english  */
		EFlush						= 50   /*!< \~russian ошибка при очистки очереди буферов \~english  */
		// 51-55 reserved
	};

	AbstractSerial(const QString &deviceName = DEFAULT_DEVICE_NAME, QObject *parent = 0);
	virtual ~AbstractSerial();

	void setDeviceName(const QString &deviceName);
	QString deviceName() const;

	bool isValid() const;

	//получить список последовательных устройств
	QStringList serialDevicesAvailable() const;

	//baud rate
	bool setBaudRate(BaudRate baudRate);
	bool setBaudRate(const QString &baudRate);
	QString baudRate() const;
	QStringList listBaudRate() const;
	//data bits
	bool setDataBits(DataBits dataBits);
	bool setDataBits(const QString &dataBits);
	QString dataBits() const;
	QStringList listDataBits() const;
	//parity
	bool setParity(Parity parity);
	bool setParity(const QString &parity);
	QString parity() const;
	QStringList listParity() const;
	//stop bits
	bool setStopBits(StopBits stopBits);
	bool setStopBits(const QString &stopBits);
	QString stopBits() const;
	QStringList listStopBits() const;
	//flow
	bool setFlowControl(Flow flow);
	bool setFlowControl(const QString &flow);
	QString flowControl() const;
	QStringList listFlowControl() const;
	//CharIntervalTimeout
	bool setCharIntervalTimeout(int msecs = 50);
	int charIntervalTimeout() const;

	bool setDtr(bool set);
	bool setRts(bool set);

	ulong lineStatus();

	// from public QIODevice
	bool open(OpenMode mode);
	void close();
	bool flush();
	bool reset();

	qint64 bytesAvailable(bool wait = false);
	bool isSequential() const;

	bool waitForReadyRead(int msecs = 5000);
	bool waitForBytesWritten(int msecs = 5000);

	//
	void enableEmitStatus(bool enable);

protected:
	AbstractSerialPrivate * const d_ptr;
	// from protected QIODevice
	qint64 readData(char *data, qint64 maxlen);
	qint64 writeData(const char *data, qint64 len);

private:
	Q_DECLARE_PRIVATE(AbstractSerial)
	Q_DISABLE_COPY(AbstractSerial)

	Q_PRIVATE_SLOT(d_func(), bool canReadNotification())
	Q_PRIVATE_SLOT(d_func(), bool canWriteNotification())
	//for a human
	QMap<AbstractSerial::Status, QString> statusesMap;
	QMap<AbstractSerial::BaudRate, QString> baudRatesMap;
	QMap<AbstractSerial::DataBits, QString> dataBitsMap;
	QMap<AbstractSerial::Parity, QString> paritysMap;
	QMap<AbstractSerial::StopBits, QString> stopBitsMap;
	QMap<AbstractSerial::Flow, QString> flowsMap;
	void initialiseMaps();

	BaudRate baudRateFromString(const QString &str) const;
	DataBits dataBitsFromString(const QString &str) const;
	Parity parityFromString(const QString &str) const;
	StopBits stopBitsFromString(const QString &str) const;
	Flow flowFromString(const QString &str) const;
	Status statusFromString(const QString &str) const;

	QString baudRateToString(BaudRate val) const;
	QString dataBitsToString(DataBits val) const;
	QString parityToString(Parity val) const;
	QString stopBitsToString(StopBits val) const;
	QString flowToString(Flow val) const;
	QString statusToString(Status val) const;

	void emitStatusString(Status status);
	bool canEmitStatusString() const;
};

class AbstractSerialEngine;
class AbstractSerialPrivate
{
	Q_DECLARE_PUBLIC(AbstractSerial)
public:
	AbstractSerialPrivate(AbstractSerial * qq);
	virtual ~AbstractSerialPrivate();

	bool canReadNotification();
	bool canWriteNotification();

	bool waitForReadyReadCalled;

	bool emittedReadyRead;
	bool emittedBytesWritten;
	bool emittedStatus;

	AbstractSerialEngine *serialEngine;

	bool initSerialLayer();
	void resetSerialLayer();

	void setupSerialNotifiers();
	void enableSerialNotifiers(bool enable);

	AbstractSerial * const q_ptr;
};

#endif // ABSTRACTSERIAL_H
