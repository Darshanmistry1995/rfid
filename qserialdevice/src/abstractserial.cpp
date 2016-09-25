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


/*! \class AbstractSerial

    \brief \~russian Класс AbstractSerial предоставляет минимальный и достаточный интерфейс для работы с последовательными устройствами. \~english Class AbstractSerial provides the minimum and adequate interface for working with serial devices.

    \reentrant

    \~russian Класс для работы с последовательными устройствами (портами).
    Этот класс является кросс - платформенной "низкоуровневой" библиотекой, которая основана на Qt4 и,
    соответственно, может использоваться для создания приложений Qt4.

    Реализация структуры этого класса во многом аналогична реализации QAbstractSocket из поставки Qt4, т.е.
    соответствует идеологии открытых/закрытых классов, а также этот класс является наследником класса QIODevice и
    соответственно наследует от него все его методы.

    Работа этого класса с последовательным устройством осуществляется в асинхронном режиме!

    При использовании в различных операционных системах - используются их родные системные API:
    \li  В ОС Windows семейств NT/2K/XP и т.д. используется Win32 API.
    \li  В POSIX подобных ОС используются системные вызовы POSIX.

    Начало работы с классом необходимо начинать с создания экземпляра объекта AbstractSerial.
    Пример:
    \code
        ...
        AbstractSerial *serialDevice = new AbstractSerial("device name");
        ...
    \endcode
    где "device name" - имя последовательного устройства, реально существующего в операционной системе.
    Тип переменной "device name" является QString. Остальные параметры устройства, такие как:
    скорость, биты данных, четность, стоп биты, контроль потоком, интервал ожидания символа (в мсек) -
    всегда принимаются по умолчанию соответственно : 9600 8 N 1 50ms !
    \note Параметр "device name" - можно не указывать,
    тогда по умолчанию объекту присвоится первое имя реально существующего в системе устройства.
    \verbatim
        Операционная система:          Имя устройства по умолчанию:
        Windows                        "COM1"
        IRIX                           "/dev/ttyf1"
        HPUX                           "/dev/tty1p0"
        SOLARIS                        "/dev/ttya"
        FREEBSD                        "/dev/ttyd1"
        LINUX                          "/dev/ttyS0"
        <default>                      "/dev/ttyS0"
    \endverbatim

    Для открытия последовательного устройства необходимо вызвать метод: bool AbstractSerial::open(QIODevice::OpenMode).
    При успешном открытии этот метод вернет true.
    Класс AbstractSerial поддерживает только опции открытия QIODevice как:
    OpenMode::ReadOnly, OpenMode::WriteOnly, OpenMode::ReadWrite,
    при этом для корректной работы необходимо добавлять режим OpenMode::Unbuffered
    Пример:
    \code
        ...
        //this example open device as ReadOnly
        bool ret = serialDevice->open(QIODevice::ReadWrite | QIODevice::Unbuffered);
        ...
    \endcode
    \note Последовательное устройство ВСЕГДА открывается с параметрами по умолчанию : 9600 8 N 1 50ms

    \sa open()

    Для закрытия последовательного устройства необходимо вызвать метод: void AbstractSerial::close()

    \sa close()

    После того, как устройство успешно открыто можно приступать к его конфигурированию.
    Для этого необходимо воспользоваться следующими методами:
    \li bool AbstractSerial::setBaudRate(AbstractSerial::BaudRate) - устанавливает скорость обмена данными.
    При успехе метод вернет true.

    \sa BaudRate, setBaudRate(), baudRate(), listBaudRate()

    \li bool AbstractSerial::setDataBits(AbstractSerial::DataBits) - устанавливает количество бит данных.
    При успехе метод вернет true.

    \sa DataBits, setDataBits(), dataBits(), listDataBits()

    \li bool AbstractSerial::setParity(AbstractSerial::Parity) - устанавливает контроль четности.
    При успехе метод вернет true.

    \sa Parity, setParity(), parity(), listParity()

    \li bool AbstractSerial::setStopBits(AbstractSerial::StopBits) - устанавливает количество стоп бит.
    При успехе метод вернет true.

    \sa StopBits, setStopBits(), stopBits(), listStopBits()

    \li bool AbstractSerial::setFlowControl(AbstractSerial::Flow) - устанавливает контроль управления потоком.
    При успехе метод вернет true.

    \sa Flow, setFlowControl(), flowControl(), listFlowControl()

    \li bool AbstractSerial::setCharIntervalTimeout(int msecs) - устанавливает время ожидания прихода символа при чтении данных.
    При успехе метод вернет true. Т.к. устройство работает в асинхронном режиме, то достаточно для уверенного
    чтения данных подобрать опытным путем только этот параметр таймаута. Этот параметр является единственным параметром
    при чтении даннх, он только зависит от скорости и выбирать/рассчитывать его можно опытным путем.
    Обычно для уверенного приема достаточна величина 50ms для всех скоростей.
    Это упрощает работу с последовательным устройством.

    \sa charIntervalTimeout()

    Для чтения/записи данных в порт рекомендуется использовать следующие методы:
    \li QByteArray AbstractSerial::read(qint64 maxSize) - читает данные из последовательного устройства.
    Метод read() наследован от QIODevice. Теоретически размер maxSize не ограничен, т.к. метод читает доступное
    количество байт частями.
    Реально в классе AbstractSerial реализован защищенный метод: qint64 readData(char *data, qint64 maxlen) в
    котором определена вся логика чтения данных из устройства. Чтение данных осуществляется таким образом, чтобы
    не терялись данные!
    \note Работа метода: QByteArray readAll() не тестировалась. Рекомендуется использовать метод read()

    \li qint64 AbstractSerial::write(const QByteArray & byteArray) - записывает данные в последовательное устройство.
    Метод write() наследован от QIODevice. Теоретически размер byteArray не ограничен, т.к. метод записывает
    данные по частям. После записи в устройство всех данных излучается сигнал bytesWritten(qint64 bytes).
    Реально в классе AbstractSerial реализован защищенный метод: qint64 writeData(const char * data, qint64 maxSize) в
    котором определена вся логика записи данных в устройство.

    \sa readData(), writeData()

    Для ожидания прихода данных в последовательное устройство используется метод: bool AbstractSerial::waitForReadyRead(int msecs).
    Метод ожидает в течении времени msecs прихода в приемный буфер последовательного устройства хотя бы одного байта данных.
    При успешном завершении ожидания (когда байт пришел до таймаута msecs) метод излучает сигнал readyRead() и
    возвращает true.

    \sa waitForReadyRead(), readyRead()

    Для очистки буферов последовательного устройства используются методы:
    \li bool AbstractSerial::flush() - принудительно ожидает завершения операций чтения/записи и по их окончании
    очищает очереди ввода/вывода
    \li bool AbstractSerial::reset() - не ожидая завершения операций ввода/вывода очищает буферы последовательного
    устройства.

    \sa flush(), reset()

    Для управления линиями DTR и RTS используются методы:
    bool AbstractSerial::setDtr(bool set) и bool AbstractSerial::setRts(bool set);
    При успешном завершении методы возвращают true.

    \sa setDtr(), setRts()

    Для получения статусов линий CTS, DSR, DCD, RI, RTS, DTR, ST, SR используется метод:
    ulong AbstractSerial::lineStatus() const.
    Метод возвращает закодированные значения статусов линий побитовым ИЛИ:
    \verbatim
        CTS  0x01
        DSR  0x02
        DCD  0x04
        RI   0x08
        RTS  0x10
        DTR  0x20
        ST   0x40
        SR   0x80
    \endverbatim

    \sa lineStatus()

    Для получения текущего количества байт в приемном буфере последовательного устройства готовых для чтения
    используется метод: qint64 AbstractSerial::bytesAvailable(bool wait = false).
    Параметр wait - означает ожидать появления в приемном буфере байт в течении времени charIntervalTimeout.
    Этот параметр добавлен специально для корректной выдачи сигнала readyRead() в цикле событий,
    метода чтения read(), а также метода ожидания waitForReadyRead() в ОС Windows.
    В POSIX ОС (Linux etc) параметр wait игнорируется.

    \sa bytesAvailable(), charIntervalTimeout(), waitForReadyRead(), read(), readyRead()

    Для включения/отключения режима излучения сигнала signalStatus() используется метод:
    void AbstractSerial:enableEmitStatus(bool enable).

    \sa enableEmitStatus(), signalStatus()

    Класс AbstractSerial реализует следующие сигналы:
    \li void AbstractSerial::readyRead() - излучается ОДИН РАЗ при приходе в приемный буфер устройства хотя бы одного байта данных.
    Событие о приходе данных контролируется в цикле сообщений.
    Если приходит следующая часть данных (или байт) - то сигнал не излучается!!! Для того, чтобы сигнал вновь излучился
    необходимо прочитать из буфера данные методом read() или сбросить буфер reset() !!!
    Также этот сигнал излучается каждый раз при успешном вызове метода waitForReadyRead().

    \sa readyRead(), read(), waitForReadyRead()

    \li void AbstractSerial::bytesWritten(qint64 bytes) - излучается при успешном завершении метода write().

    \sa bytesWritten(), write()

    \li void AbstractSerial::signalStatus(const QString &status, QDateTime current) - несет информацию о текущем статусе последовательного
    устройства, а также времени и дате возникновения статуса. Излучается всякий раз при следующих действиях:
        \li при выполнении методов open(), close()
        \li при ошибках при конфигурировании
        \li при ошибках чтении/записи и т.д.
    Излучение сигнала можно включать/отключать при вызове enableEmitStatus(true/false).

    \sa signalStatus(), Status, enableEmitStatus()


    ----------------------------------------------------------------------------------------------------------------------------------------------
    \warning
    \li В этом описании перечислены все реализованные в классе методы.
    Если же не указан реализованный метод - то значит его значение несущественно.
    \li В текущей версии класса он не поддерживает нотификацию записи, хотя код её реализации имеется в исходных кодах.
    Нотификация записи отключена! (т.е. части кода закомментированы)
    \li По умолчанию класс сконфигурирован таким образом, чтобы записывать в консоль отладочные сообщения.
    Для отключения вывода этих сообщений в консоль - необходимо закомментировать в *.cpp файлах строчки вида:

    \~english class to work with serial devices (ports).
    This class is a cross - platform low-level library, which is based on Qt4, and
    respectively, can be used to create applications Qt4.

    Implementation of this class structure is broadly similar to the implementation of the supply QAbstractSocket Qt4, ie
    corresponds to the ideology of public/private classes, and this class is a descendant of QIODevice and
    respectively inherit from him all his methods.

    The work of this class with a serial device is in asynchronous mode!

    When used in a variety of operating systems - used their native system API:
    \li in Windows NT/2K/XP family, etc. use Win32 API.
    \li In POSIX like OS system calls used by POSIX.

    Getting Started with the class should begin with the creation of an object instance AbstractSerial.
    Example:
    \code
        ...
        AbstractSerial * serialDevice = new AbstractSerial("device name");
        ...
    \endcode
    where the "device name" - name of the serial device, truly present in the operating system.
    Type variable "device name" is a QString. The other device parameters, such as:
    speed, data bits, parity, stop bits, flow control, timed character (in milliseconds) --
    always accepted by default, respectively: 9600 8 N 1 50ms!
    \note parameter "device name" - you can not specify
    then by default be assigned to the object first name actually exists in the system device.
    \verbatim
        Operating system:          The device name default:
        Windows                    "COM1"
        IRIX                       "/dev/ttyf1"
        HPUX                       "/dev/tty1p0"
        SOLARIS                    "/dev/ttya"
        FREEBSD                    "/dev/ttyd1"
        LINUX                      "/dev/ttyS0"
        <default>                  "/dev/ttyS0"
    \endverbatim

    To open a serial device, you must call the method: bool AbstractSerial::open(QIODevice::OpenMode).
    If successful, the opening of this method returns true.
    Class AbstractSerial supports only option open QIODevice as:
    OpenMode::ReadOnly, OpenMode::WriteOnly, OpenMode::ReadWrite,
    while for the correct operation mode to add OpenMode::Unbuffered
    Example:
    \code
        ...
        //this example open device as ReadOnly
        bool ret = serialDevice->open(QIODevice::ReadWrite | QIODevice::Unbuffered);
        ...
    \endcode
    \note the serial device always opens with the default settings: 9600 8 N 1 50ms

    \sa open()

    To close the serial device need to call the method: void AbstractSerial::close()

    \sa close()

    Once the device successfully opened, you are ready to configure it.
    To do this, use the following methods:
    \li bool AbstractSerial::setBaudRate(AbstractSerial::BaudRate) - sets the speed of data exchange.
    With the success of the method returns true.

    \sa BaudRate, setBaudRate(), baudRate(), listBaudRate()

    \li bool AbstractSerial::setDataBits(AbstractSerial::DataBits) - sets the number of bits of data.
    With the success of the method returns true.

    \sa DataBits, setDataBits(), dataBits(), listDataBits()

    \li bool AbstractSerial::setParity(AbstractSerial::Parity) - sets the parity.
    With the success of the method returns true.

    \sa Parity, setParity(), parity(), listParity()

    \li bool AbstractSerial::setStopBits(AbstractSerial::StopBits) - sets the number of stop bits.
    With the success of the method returns true.

    \sa StopBits, setStopBits(), stopBits(), listStopBits()

    \li bool AbstractSerial::setFlowControl(AbstractSerial::Flow) - sets the control flow control.
    With the success of the method returns true.

    \sa Flow, setFlowControl(), flowControl(), listFlowControl()

    \li bool AbstractSerial::setCharIntervalTimeout(int msecs) - sets the waiting time of arrival character when reading data.
    With the success of the method returns true. Since device operates in asynchronous mode, it is enough for sure
    reading the data to select only experienced by this parameter timeout. This option is the only parameter
    reading dannh, it just depends on the speed and choose/calculate it can be empirically.
    Typically, for sure receive sufficient magnitude 50ms for all speeds.
    It simplifies work with a serial device.

    \sa charIntervalTimeout()

    To read/write data to the port is recommended to use the following methods:
    \li QByteArray AbstractSerial::read(qint64 maxSize) - reads data from a serial device.
    Method read() inherited from QIODevice. Theoretically, the size maxSize is not limited, because method reads the available
    number of bytes in parts.
    Really class AbstractSerial implemented a secure method: qint64 readData(char * data, qint64 maxlen) in
    which defines all the logic of reading data from the device. Reading data is carried out in such a way that
    not lose data!
    \note work method: QByteArray readAll() not tested. It is recommended to use the method read()

    \li qint64 AbstractSerial::write(const QByteArray & byteArray) - writes data to the serial device.
    Method write() inherited from QIODevice. Theoretically, the size byteArray is not limited, because method records
    data in parts. After recording all data in the device emitted a signal bytesWritten(qint64 bytes).
    Really class AbstractSerial implemented a secure method: qint64 writeData(const char * data, qint64 maxSize) in
    which defines all the logic of writing data to the device.

    \sa readData(), writeData()

    In anticipation of the return data in the serial device using the method: bool AbstractSerial::waitForReadyRead(int msecs).
    The method waits for the arrival time msecs in the receive buffer of the serial device at least one byte of data.
    If successful, the expectation (when B came to a timeout msecs) method emits a signal readyRead() and
    returns true.

    \sa waitForReadyRead(), readyRead()

    To clear the buffer of the serial device used methods:
    \li bool AbstractSerial::flush() - forced forward to the completion of read/write and at their end
    clears the I/O queues
    \li bool AbstractSerial::reset() - not waiting for the completion of I/O clears the serial buffer
    device.

    \sa flush(), reset()

    To control lines DTR and RTS methods are used:
    bool AbstractSerial::setDtr(bool set) and bool AbstractSerial::setRts(bool set);
    If successful, the method returns true.

    \sa setDtr(), setRts()

    For the status lines CTS, DSR, DCD, RI, RTS, DTR, ST, SR method is used:
    ulong AbstractSerial::lineStatus() const.
    The method returns an encoded value status lines bitwise OR:
    \verbatim
        CTS 0x01
        DSR 0x02
        DCD 0x04
        RI 0x08
        RTS 0x10
        DTR 0x20
        ST 0x40
        SR 0x80
    \endverbatim

    \sa lineStatus()

    For the current number of bytes in the receive buffer the serial device is ready for reading
    method is used: qint64 AbstractSerial::bytesAvailable(bool wait = false).
    Parameter wait - means expect to see in the receive buffer of bytes in the flow of time charIntervalTimeout.
    This parameter was added specifically for the correct issuance readyRead() signal in a cycle of events,
    read method read(), as well as the method of expectations waitForReadyRead() in Windows.
    In POSIX OS (Linux etc) the wait option is ignored.

    \sa bytesAvailable(), charIntervalTimeout(), waitForReadyRead(), read(), readyRead()

    To enable/disable the radiation signal signalStatus() method is used:
    void AbstractSerial::enableEmitStatus(bool enable).

    \sa enableEmitStatus(), signalStatus()

    Class AbstractSerial implements the following signals:
    \li void AbstractSerial::readyRead() - emitted once at the arrival in the receive buffer device at least one byte of data.
    Event of the arrival of data is controlled in the event loop.
    When comes the next part of the data (or B) - the signal is not radiated! In order to re-emitted signal
    must read the data from the buffer by the method read() or reset the buffer reset()!!!
    Also, this signal is emitted each time a successful call to the method waitForReadyRead().

    \sa readyRead(), read(), waitForReadyRead()

    \li void AbstractSerial::bytesWritten(qint64 bytes) - emitted when the successful completion of the method write().

    \sa bytesWritten(), write()

    \li void AbstractSerial::signalStatus (const QString & status, QDateTime current) - carries information about the current status of the serial
    devices, as well as time and date of status. Is emitted whenever the following actions:
        \li in the performance of methods open(), close()
        \li for errors in configuring
        \li error when reading/writing, etc.
    The emission signal can be switched on/off when you call enableEmitStatus(true/false).

    \sa signalStatus(), Status, enableEmitStatus()


    ----------------------------------------------------------------------------------------------------------------------------------------------
    \warning
    \li In this description lists all the methods implemented in the classroom.
    If you do not specify the method implemented - that means its value is immaterial.
    \li In the current version of the class does not support write notification, although the code of its implementation is available in source code.
    Notification write off! (ie part of the code commented out)
    \li The default class is configured in such a way as to write to the console debug messages.
    To disable these messages in the console - need to comment out in the *. cpp files, a line like:

    \~
    \code
        ...
        //#define ABSTRACTSERIAL_DEBUG
        ...
    \endcode

    \version \~ 0.1.0

    \author \~ Shienkov Denis \li ICQ: 321789831 \li e-mail: scapig2@yandex.ru

    \~russian
    \note \li В версии 0.1.0. работа библиотеки тестировалась только с : Windows XP SP3 Pro и ArchLinux x86-64 Summer. \li Библиотекой теоретически должны поддерживаться такие ОС как: все семейство Windows NT/2k/XP , а также POSIX совместимые.
    \~english
    \note \li In a version 0.1.0. work of library was tested only with : Windows XP Sp3 Pro and Archlinux x86-64 Summer. \li In theory must a library be supported such OS as: all of family of Windows Nt/2k/xp, and also POSIX compatible.
*/

#include <QStringList>
#include <QAbstractEventDispatcher>
#include <QTime>

#include "abstractserialengine.h"
#include "datatypes.h"

#define ABSTRACTSERIAL_DEBUG

#ifdef ABSTRACTSERIAL_DEBUG
#include <QDebug>
#include <QString>
#include <ctype.h>
#endif



AbstractSerialPrivate::AbstractSerialPrivate(AbstractSerial * qq)
	: q_ptr(qq),
	waitForReadyReadCalled(false),
	emittedReadyRead(false),
	emittedBytesWritten(false),
	emittedStatus(false),
	serialEngine(0)
{
}

AbstractSerialPrivate::~AbstractSerialPrivate()
{
}

void AbstractSerialPrivate::resetSerialLayer()
{
	if (serialEngine) {
		delete serialEngine;
		serialEngine = 0;
	}
}

bool AbstractSerialPrivate::initSerialLayer()
{
	Q_Q(AbstractSerial);

	serialEngine = AbstractSerialEngine::createSerialEngine(q);
	if (serialEngine) return true;

	return false;
}

void AbstractSerialPrivate::setupSerialNotifiers()
{
	Q_Q(AbstractSerial);

	if (serialEngine) {
		switch ((q->openMode()) & (~QIODevice::Unbuffered)) {
			case QIODevice::ReadOnly :
				serialEngine->setReadNotificationEnabled(true);
				QObject::connect(serialEngine, SIGNAL(readNotification()), q, SLOT(canReadNotification()));
				break;
			case QIODevice::WriteOnly :
				/*
				serialEngine->setWriteNotificationEnabled(true);
				QObject::connect(serialEngine, SIGNAL(writeNotification()), q, SLOT(canWriteNotification()));
				*/
				break;
			case QIODevice::ReadWrite :
				serialEngine->setReadNotificationEnabled(true);
				QObject::connect(serialEngine, SIGNAL(readNotification()), q, SLOT(canReadNotification()));
				/*
				serialEngine->setWriteNotificationEnabled(true);
				QObject::connect(serialEngine, SIGNAL(writeNotification()), q, SLOT(canWriteNotification()));
				*/
				break;
			default:
				;
		}
	}
}

void AbstractSerialPrivate::enableSerialNotifiers(bool enable)
{
	if (serialEngine) {
		serialEngine->setReadNotificationEnabled(enable);
		//serialEngine->setWriteNotificationEnabled(enable);
	}
}

bool AbstractSerialPrivate::canReadNotification()
{
	Q_Q(AbstractSerial);
	if (waitForReadyReadCalled) {
		emittedReadyRead = true;
		emit q->readyRead();
		waitForReadyReadCalled = false;
		return true;
	}

	if (!emittedReadyRead) {
		emittedReadyRead = true;
		emit q->readyRead();
		return true;
	}

	return false;
}

bool AbstractSerialPrivate::canWriteNotification()
{
/*
    In this version library is not used.
*/
	return false;
}

//----------------------------------------------------------------------------------------------------------------------------------------

/*!
\fn AbstractSerial::AbstractSerial(const QString &deviceName, QObject *parent)
\~russian
Конструктор по умолчанию. Создает объект, ассоциированный с реальным последовательным устройством в системе.
\param[in] deviceName - имя последовательного устройства, реально имеющегося в системе.
Например: для Windows - это COM1, COM2 ... COMn, для Linux - это /dev/ttyS0, /dev/ttyS1 и т.д.
\note  Если не указать (пропустить) в конструкторе параметр deviceName - то по умолчанию устройству присвоится имя = DEFAULT_DEVICE_NAME
\~english
Default constructor. Creates an object, associated with the real serial device in the system.
\param[in] deviceName - serial device, really present in the system name.
For example: for Windows - it COM1, COM2 ... COMn, for Linux - it /dev/ttyS0, /dev/ttyS1 etc
\note If not to specify (to skip) the parameter of devicename in a constructor - that by default the name will be appropriated a device = DEFAULT_DEVICE_NAME
\~
\verbatim
Operating system:        Name DEFAULT_DEVICE_NAME:
  Windows                        "COM1"
  IRIX                           "/dev/ttyf1"
  HPUX                           "/dev/tty1p0"
  SOLARIS                        "/dev/ttya"
  FREEBSD                        "/dev/ttyd1"
  LINUX                          "/dev/ttyS0"
  <default>                      "/dev/ttyS0"
\endverbatim
*/
AbstractSerial::AbstractSerial(const QString &deviceName, QObject *parent)
	: QIODevice(parent), d_ptr(new AbstractSerialPrivate(this))
{
	Q_D(AbstractSerial);

	if (!d->initSerialLayer()) return;
	d->serialEngine->setDeviceName(deviceName);
	initialiseMaps();
}

/*!
\fn AbstractSerial::~AbstractSerial()
\~russian Деструктор по умолчанию. \~english Default destructor.
*/
AbstractSerial::~AbstractSerial()
{
	Q_D(AbstractSerial);
	d->resetSerialLayer();
	delete d_ptr;
}

/*!
\fn void AbstractSerial::initialiseMaps()
\private
\internal
\~russian Инициализирует карты трансляции значений статусов, скоростей, и т.д. Вызывается автоматически при инициализации класса.
\~english Initializes the cards of translation of values of statuses, speeds, etc. Caused automatically during initializing of class
*/
void AbstractSerial::initialiseMaps()
{
	//filling statusesMap
	statusesMap[AbstractSerial::ENone] = QObject::tr("No errors.");
	statusesMap[AbstractSerial::ENoneOpen] = QObject::tr("Opened::Device is successfully opened. OK!");
	statusesMap[AbstractSerial::ENoneClose] = QObject::tr("Closed::Device is successfully closed. OK!");
	statusesMap[AbstractSerial::ENoneSetBaudRate] = QObject::tr("Controls::Baud rate is successfully set. OK!");
	statusesMap[AbstractSerial::ENoneSetParity] = QObject::tr("Controls::Parity is successfully set. OK!");
	statusesMap[AbstractSerial::ENoneSetDataBits] = QObject::tr("Controls::Data bits is successfully set. OK!");
	statusesMap[AbstractSerial::ENoneSetStopBits] = QObject::tr("Controls::Stop bits is successfully set. OK!");
	statusesMap[AbstractSerial::ENoneSetFlow] = QObject::tr("Controls::Flow is successfully set. OK!");
	statusesMap[AbstractSerial::ENoneSetCharTimeout] = QObject::tr("Controls::Char timeout is successfully set. OK!");
	statusesMap[AbstractSerial::EDeviceIsNotOpen] = QObject::tr("Device is not open. Error!");
	statusesMap[AbstractSerial::EOpenModeUnsupported] = QObject::tr("Opened::Opened mode unsupported. Error!");
	statusesMap[AbstractSerial::EOpenModeUndefined] = QObject::tr("Opened::Opened mode undefined. Error!");
	statusesMap[AbstractSerial::EOpenInvalidFD] = QObject::tr("Opened::Invalid device descriptor. Error!");
	statusesMap[AbstractSerial::EOpenOldSettingsNotSaved] = QObject::tr("Opened::Fail saved old settings. Error!");
	statusesMap[AbstractSerial::EOpenGetCurrentSettings] = QObject::tr("Opened::Fail get current settings. Error!");
	statusesMap[AbstractSerial::EOpenSetDefaultSettings] = QObject::tr("Opened::Fail set default settings. Error!");
	statusesMap[AbstractSerial::ECloseSetOldSettings] = QObject::tr("Closed::Fail set old settings. Error!");
	statusesMap[AbstractSerial::ECloseFD] = QObject::tr("Closed::Fail close device descriptor. Error!");
	statusesMap[AbstractSerial::EClose] = QObject::tr("Closed::Fail close device. Error!");
	statusesMap[AbstractSerial::ESetBaudRate] = QObject::tr("Parameters::Set baud rate fail. Error!");
	statusesMap[AbstractSerial::ESetDataBits] = QObject::tr("Parameters::Set data bits fail. Error!");
	statusesMap[AbstractSerial::ESetParity] = QObject::tr("Parameters::Set parity fail. Error!");
	statusesMap[AbstractSerial::ESetStopBits] = QObject::tr("Parameters::Set stop bits fail. Error!");
	statusesMap[AbstractSerial::ESetFlowControl] = QObject::tr("Parameters::Set flow control fail. Error!");
	statusesMap[AbstractSerial::ESetCharIntervalTimeout] = QObject::tr("Parameters::Set char interval timeout. Error!");
	statusesMap[AbstractSerial::EBytesAvailable] = QObject::tr("Controls::Get bytes available fail. Error!");
	statusesMap[AbstractSerial::ESetDtr] = QObject::tr("Controls::Set DTR fail. Error!");
	statusesMap[AbstractSerial::ESetRts] = QObject::tr("Controls::Set RTS fail. Error!");
	statusesMap[AbstractSerial::ELineStatus] = QObject::tr("Controls::Get lines status fail. Error!");
	statusesMap[AbstractSerial::EWaitReadyReadIO] = QObject::tr("Controls::Wait for ready read from device - i/o problem. Error!");
	statusesMap[AbstractSerial::EWaitReadyReadTimeout] = QObject::tr("Controls::Wait for ready read timeout. Error!");
	statusesMap[AbstractSerial::EWaitReadyWriteIO] = QObject::tr("Controls::Wait for bytes writtten to device - i/o problem. Error!");
	statusesMap[AbstractSerial::EWaitReadyWriteTimeout] = QObject::tr("Controls::Wait for bytes writtten timeout. Error!");
	statusesMap[AbstractSerial::EReadDataIO] = QObject::tr("Controls::Read data from device - i/o problem. Error!");
	statusesMap[AbstractSerial::EWriteDataIO] = QObject::tr("Controls::Write data to device - i/o problem. Error!");
	statusesMap[AbstractSerial::EFlush] = QObject::tr("Controls::Flush fail. Error!");
	//filling baudRatesMap
	baudRatesMap[AbstractSerial::BaudRate50]=QObject::tr("50 baud");
	baudRatesMap[AbstractSerial::BaudRate75]=QObject::tr("75 baud");
	baudRatesMap[AbstractSerial::BaudRate110]=QObject::tr("110 baud");
	baudRatesMap[AbstractSerial::BaudRate134]=QObject::tr("134 baud");
	baudRatesMap[AbstractSerial::BaudRate150]=QObject::tr("150 baud");
	baudRatesMap[AbstractSerial::BaudRate200]=QObject::tr("200 baud");
	baudRatesMap[AbstractSerial::BaudRate300]=QObject::tr("300 baud");
	baudRatesMap[AbstractSerial::BaudRate600]=QObject::tr("600 baud");
	baudRatesMap[AbstractSerial::BaudRate1200]=QObject::tr("1200 baud");
	baudRatesMap[AbstractSerial::BaudRate1800]=QObject::tr("1800 baud");
	baudRatesMap[AbstractSerial::BaudRate2400]=QObject::tr("2400 baud");
	baudRatesMap[AbstractSerial::BaudRate4800]=QObject::tr("4800 baud");
	baudRatesMap[AbstractSerial::BaudRate9600]=QObject::tr("9600 baud");
	baudRatesMap[AbstractSerial::BaudRate14400]=QObject::tr("14400 baud");
	baudRatesMap[AbstractSerial::BaudRate19200]=QObject::tr("19200 baud");
	baudRatesMap[AbstractSerial::BaudRate38400]=QObject::tr("38400 baud");
	baudRatesMap[AbstractSerial::BaudRate56000]=QObject::tr("56000 baud");
	baudRatesMap[AbstractSerial::BaudRate57600]=QObject::tr("57600 baud");
	baudRatesMap[AbstractSerial::BaudRate76800]=QObject::tr("76800 baud");
	baudRatesMap[AbstractSerial::BaudRate115200]=QObject::tr("115200 baud");
	baudRatesMap[AbstractSerial::BaudRate128000]=QObject::tr("128000 baud");
	baudRatesMap[AbstractSerial::BaudRate256000]=QObject::tr("256000 baud");
	//filling dataBitsMap
	dataBitsMap[AbstractSerial::DataBits5]=QObject::tr("5 bit");
	dataBitsMap[AbstractSerial::DataBits6]=QObject::tr("6 bit");
	dataBitsMap[AbstractSerial::DataBits7]=QObject::tr("7 bit");
	dataBitsMap[AbstractSerial::DataBits8]=QObject::tr("8 bit");
	//filling paritysMap
	paritysMap[AbstractSerial::ParityNone]=QObject::tr("None");
	paritysMap[AbstractSerial::ParityOdd]=QObject::tr("Odd");
	paritysMap[AbstractSerial::ParityEven]=QObject::tr("Even");
	paritysMap[AbstractSerial::ParityMark]=QObject::tr("Mark");
	paritysMap[AbstractSerial::ParitySpace]=QObject::tr("Space");
	//filling stopBitsMap
	stopBitsMap[AbstractSerial::StopBits1]=QObject::tr("1");
	stopBitsMap[AbstractSerial::StopBits1_5]=QObject::tr("1.5");
	stopBitsMap[AbstractSerial::StopBits2]=QObject::tr("2");
	//filling flowsMap
	flowsMap[AbstractSerial::FlowControlOff]=QObject::tr("Disable");
	flowsMap[AbstractSerial::FlowControlHardware]=QObject::tr("Hardware");
	flowsMap[AbstractSerial::FlowControlXonXoff]=QObject::tr("Xon/Xoff");
}

/*!
\fn bool AbstractSerial::isValid() const
\public
*/
bool AbstractSerial::isValid() const
{
	Q_D(const AbstractSerial);
	return d->serialEngine ? true : false;
}

/*!
\fn void AbstractSerial::setDeviceName(const QString &deviceName)
\public
\~russian Присваивает созданному объекту имя последовательного устройства, с которым необходимо работать.
\param[in] deviceName - имя последовательного устройства, реально имеющегося в системе.
Например: для Windows - это COM1, COM2 ... COMn, для Linux - это /dev/ttyS0, /dev/ttyS1 и т.д.
\note Использовать этот метод необходимо только ПЕРЕД открытием устройства!
\~english Appropriates the created object the serial device with which it is necessary to work name.
\param[in] deviceName - serial device, really present in the system name.
For example: for Windows - it COM1, COM2 ... COMn, for Linux - it /dev/ttyS0, /dev/ttyS1 etc
\note  Uses this method is necessary only BEFORE opening of device!
*/
void AbstractSerial::setDeviceName(const QString &deviceName)
{
	Q_D(AbstractSerial);
	if (isValid()) d->serialEngine->setDeviceName(deviceName);
}

/*!
\fn QString AbstractSerial::deviceName() const
\public
\~russian Возвращает имя последовательного устройства, с которым сконфигурирован объект.
\return \li Имя последовательного устройства, с которым сконфигурирован объект в виде QString \li 0 в случае ошибки
\note Использовать этот метод можно в любой момент после создания объекта, т.е. неважно открыто реально устройство или нет.
\~english Returns the serial device which an object is configured with name.
\return \li Serial device with which an object is configured as Qstring name \li 0 in the case of error
\note Uses this method is possible at any moment after creation of object, I.e. it is unimportant openly really device or not.
*/
QString AbstractSerial::deviceName() const
{
	Q_D(const AbstractSerial);
	return (isValid()) ? d->serialEngine->deviceName() : 0;
}

/*!
\fn QStringList AbstractSerial::serialDevicesAvailable() const
\~russian Получает список имен последовательных устройств, имеющихся в системе.
\return \li Список имен последовательных устройств в виде QStringList \li 0 - в случае ошибки
\note \li Тестировалось только под Windows XP и ArchLinux x86-64 Summer \li Использовать этот метод можно в любой момент после создания объекта, т.е. неважно открыто реально устройство или нет.
\~english Gets the list of the names of serial devices, present in the system.
\return \li List of the names of serial devices as Qstringlist \ 0 - in the case of error
\note \li Tested only under Windows XP and Archlinux x86-64 Summer \li Uses this method is possible at any moment after creation of object, I.e. it is unimportant openly really device or not.
*/
QStringList AbstractSerial::serialDevicesAvailable() const
{
	Q_D(const AbstractSerial);
	return (isValid()) ? d->serialEngine->serialDevicesAvailable() : (QStringList)0;
}

/*!
\fn bool AbstractSerial::setBaudRate(BaudRate baudRate)
\public
\~russian Устанавливает для последовательного устройства скорость в бодах.
\param[in] baudRate - желаемая скорость последовательного устройства в бодах из перечисления . \sa AbstractSerial::BaudRate
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр baudRate будет игнорироваться)
\~english Sets a baud rate for a serial device
\param[in] baudRate -  desired baud rate of serial device from enumeration . \sa AbstractSerial::BaudRate
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of BaudRate will be ignored otherwise)
*/
bool AbstractSerial::setBaudRate(BaudRate baudRate)
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->setBaudRate(baudRate);
		(ret) ? emitStatusString(ENoneSetBaudRate) : emitStatusString(ESetBaudRate);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn bool AbstractSerial::setBaudRate(const QString &baudRate)
\public
\overload
\~russian Устанавливает для последовательного устройства скорость в бодах.
\param[in] baudRate - желаемая скорость последовательного устройства в бодах в виде строки.
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр baudRate будет игнорироваться)
\~english Sets a baud rate for a serial device
\param[in] baudRate -  desired baud rate of serial device as an string
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of BaudRate will be ignored otherwise)
*/
bool AbstractSerial::setBaudRate(const QString &baudRate)
{
	return setBaudRate(baudRateFromString(baudRate));
}

/*!
\fn QString AbstractSerial::baudRate() const
\public
\~russian Возвращает скорость в ботах в виде строки с которой сконфигурировано последовательное устройство.
\return \li Скорость в виде строки QString \li 0 - в случае ошибки
\~english  Returns baud rate as a string which a serial device is configured with.
\return \li baud rate as an string QString \li 0 - in the case of error
*/
QString AbstractSerial::baudRate() const
{
	Q_D(const AbstractSerial);
	return (isValid()) ? baudRateToString(d->serialEngine->baudRate()) : 0;
}

/*!
\fn QStringList AbstractSerial::listBaudRate() const
\public
\~russian Возвращает текстовый список всех скоростей, поддерживаемых классом AbstractSerial.
\return Список скоростей в виде QStringList
\~english Returns the text list of all of speeds (baud rates), supported the class of Abstractserial.
\return List of speeds (baud rates) as Qstringlist
*/
QStringList AbstractSerial::listBaudRate() const
{
	return baudRatesMap.values();
}

/*!
\fn bool AbstractSerial::setDataBits(DataBits dataBits)
\public
\~russian Устанавливает для последовательного устройства количество бит данных.
\param[in] dataBits - желаемое количество бит данных последовательного устройства из перечисления . \sa AbstractSerial::DataBits
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр dataBits будет игнорироваться)
\~english Sets a data bits for a serial device
\param[in] dataBits - desired data bits of serial device from enumeration . \sa AbstractSerial::DataBits
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of dataBits will be ignored otherwise)
*/
bool AbstractSerial::setDataBits(DataBits dataBits)
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->setDataBits(dataBits);
		(ret) ? emitStatusString(ENoneSetDataBits) : emitStatusString(ESetDataBits);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn bool AbstractSerial::setDataBits(const QString &dataBits)
\public
\overload
\~russian Устанавливает для последовательного устройства количество бит данных.
\param[in] dataBits - желаемое количество бит данных последовательного устройства в виде строки
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр dataBits будет игнорироваться)
\~english Sets a data bits for a serial device
\param[in] dataBits -  desired data bits of serial device as an string
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of dataBits will be ignored otherwise)
*/
bool AbstractSerial::setDataBits(const QString &dataBits)
{
	return setDataBits(dataBitsFromString(dataBits));
}

/*!
\fn QString AbstractSerial::dataBits() const
\public
\~russian Возвращает количество бит данных в виде строки с которым сконфигурировано последовательное устройство.
\return \li Количество бит данных в виде строки QString \li 0 - в случае ошибки
\~english  Returns data bits as a string which a serial device is configured with.
\return \li data bits as an string QString \li 0 - in the case of error
*/
QString AbstractSerial::dataBits() const
{
	Q_D(const AbstractSerial);
	return (isValid()) ? dataBitsToString(d->serialEngine->dataBits()) : 0;
}

/*!
\fn QStringList AbstractSerial::listDataBits() const
\public
\~russian Возвращает текстовый список всех типов бит данных, поддерживаемых классом AbstractSerial.
\return Список бит данных в виде QStringList
\~english Returns the text list of all of data bits, supported the class of Abstractserial.
\return List of data bits as Qstringlist
*/
QStringList AbstractSerial::listDataBits() const
{
	return dataBitsMap.values();
}

/*!
\fn bool AbstractSerial::setParity(Parity parity)
\public
\~russian Устанавливает для последовательного устройства тип контроля четности.
\param[in] parity - желаемый тип контроля четности последовательного устройства из перечисления . \sa AbstractSerial::Parity
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр parity будет игнорироваться)
\~english Sets a parity for a serial device
\param[in] parity - desired parity of serial device from enumeration . \sa AbstractSerial::Parity
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of parity will be ignored otherwise)
*/
bool AbstractSerial::setParity(Parity parity)
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->setParity(parity);
		(ret) ? emitStatusString(ENoneSetParity) : emitStatusString(ESetParity);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn bool AbstractSerial::setParity(const QString &parity)
\public
\overload
\~russian Устанавливает для последовательного устройства тип контроля четности.
\param[in] parity - желаемый тип контроля четности последовательного устройства в виде строки
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр parity будет игнорироваться)
\~english Sets a parity for a serial device
\param[in] parity -  desired parity of serial device as an string
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of parity will be ignored otherwise)
*/
bool AbstractSerial::setParity(const QString &parity)
{
	return setParity(parityFromString(parity));
}

/*!
\fn QString AbstractSerial::parity() const
\public
\~russian Возвращает тип контроля четности в виде строки с которым сконфигурировано последовательное устройство.
\return \li Тип контроля четности в виде строки QString \li 0 - в случае ошибки
\~english  Returns parity as a string which a serial device is configured with.
\return \li parity as an string QString \li 0 - in the case of error
*/
QString AbstractSerial::parity() const
{
	Q_D(const AbstractSerial);
	return (isValid()) ? parityToString(d->serialEngine->parity()) : 0;
}

/*!
\fn QStringList AbstractSerial::listParity() const
\public
\~russian Возвращает текстовый список всех типов контроля четности , поддерживаемых классом AbstractSerial.
\return Список контроля четности в виде QStringList
\~english Returns the text list of all of paritys, supported the class of Abstractserial.
\return List of paritys as Qstringlist
*/
QStringList AbstractSerial::listParity() const
{
	return paritysMap.values();
}

/*!
\fn bool AbstractSerial::setStopBits(StopBits stopBits)
\public
\~russian Устанавливает для последовательного устройства количество стоп-бит.
\param[in] stopBits - желаемое количество стоп-бит последовательного устройства из перечисления . \sa AbstractSerial::StopBits
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр stopBits будет игнорироваться)
\~english Sets a stop bits for a serial device
\param[in] stopBits - desired stop bits of serial device from enumeration . \sa AbstractSerial::StopBits
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of stopBits will be ignored otherwise)
*/
bool AbstractSerial::setStopBits(StopBits stopBits)
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->setStopBits(stopBits);
		(ret) ? emitStatusString(ENoneSetStopBits) : emitStatusString(ESetStopBits);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn bool AbstractSerial::setStopBits(const QString &stopBits)
\public
\overload
\~russian Устанавливает для последовательного устройства количество стоп-бит.
\param[in] stopBits - желаемое количество стоп-бит последовательного устройства в виде строки
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр stopBits будет игнорироваться)
\~english Sets a stop bits for a serial device
\param[in] stopBits -  desired stop bits of serial device as an string
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of stopBits will be ignored otherwise)
*/
bool AbstractSerial::setStopBits(const QString &stopBits)
{
	return setStopBits(stopBitsFromString(stopBits));
}

/*!
\fn QString AbstractSerial::stopBits() const
\public
\~russian Возвращает количество стоп-бит в виде строки с которым сконфигурировано последовательное устройство.
\return \li Количество стоп-бит в виде строки QString \li 0 - в случае ошибки
\~english  Returns stop bits as a string which a serial device is configured with.
\return \li stop bits as an string QString \li 0 - in the case of error
*/
QString AbstractSerial::stopBits() const
{
	Q_D(const AbstractSerial);
	return (isValid()) ? stopBitsToString(d->serialEngine->stopBits()) : 0;
}

/*!
\fn QStringList AbstractSerial::listStopBits() const
\public
\~russian Возвращает текстовый список всех типов стоп-бит, поддерживаемых классом AbstractSerial.
\return Список стоп-бит в виде QStringList
\~english Returns the text list of all of stop bits, supported the class of Abstractserial.
\return List of stop bits as Qstringlist
*/
QStringList AbstractSerial::listStopBits() const
{
	return stopBitsMap.values();
}

/*!
\fn bool AbstractSerial::setFlowControl(Flow flow)
\public
\~russian Устанавливает для последовательного устройства режим управления потоком.
\param[in] flow - желаемый тип управления потоком последовательного устройства из перечисления . \sa AbstractSerial::Flow
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр flow будет игнорироваться)
\~english Sets a flow control for a serial device
\param[in] flow - desired flow control of serial device from enumeration . \sa AbstractSerial::Flow
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of flow will be ignored otherwise)
*/
bool AbstractSerial::setFlowControl(Flow flow)
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->setFlowControl(flow);
		(ret) ? emitStatusString(ENoneSetFlow) : emitStatusString(ESetFlowControl);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn bool AbstractSerial::setFlowControl(const QString &flow)
\public
\overload
\~russian Устанавливает для последовательного устройства режим управления потоком.
\param[in] flow - желаемый тип управления потоком последовательного устройства в виде строки
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр flow будет игнорироваться)
\~english Sets a flow control bits for a serial device
\param[in] flow -  desired flow of serial device as an string
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of flow will be ignored otherwise)
*/
bool AbstractSerial::setFlowControl(const QString &flow)
{
	return setFlowControl(flowFromString(flow));
}

/*!
\fn QString AbstractSerial::flowControl() const
\public
\~russian Возвращает режим управления потоком в виде строки с которым сконфигурировано последовательное устройство.
\return \li Режим управления потоком в виде строки QString \li 0 - в случае ошибки
\~english  Returns flow control as a string which a serial device is configured with.
\return \li flow control as an string QString \li 0 - in the case of error
*/
QString AbstractSerial::flowControl() const
{
	Q_D(const AbstractSerial);
	return (isValid()) ? flowToString(d->serialEngine->flow()) : 0;
}

/*!
\fn QStringList AbstractSerial::listFlowControl() const
\public
\~russian Возвращает текстовый список всех режимов управления потоком, поддерживаемых классом AbstractSerial.
\return Список режимов управления потоком в виде QStringList
\~english Returns the text list of all of flow controls, supported the class of Abstractserial.
\return List of flow controls as Qstringlist
*/
QStringList AbstractSerial::listFlowControl() const
{
	return flowsMap.values();
}

/*!
\fn bool AbstractSerial::setCharIntervalTimeout(int msecs)
\public
\~russian Устанавливает время ожидания прихода символа в приемный буфер последовательного устройства.
\param[in] msecs - желаемое время ожидания прихода символа, в мсек
\return \li true - при успехе \li false - в случае ошибки
\note Использовать этот метод можно ТОЛЬКО ПОСЛЕ ТОГО КАК УСТРОЙСТВО ОТКРЫЛИ! (иначе параметр msecs будет игнорироваться)
\~english Fixes time expectation of arrival of character in the receiving buffer of serial device.
\param[in] msecs - desired time of expectation of arrival of character, in msek
\return \li true - at success  \li false - at an error
\note Uses this method is possible ONLY Since DEVICE was OPENED! (the parameter of msecs will be ignored otherwise)
*/
bool AbstractSerial::setCharIntervalTimeout(int msecs)
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->setCharIntervalTimeout(msecs);
		(ret) ? emitStatusString(ENoneSetCharTimeout) : emitStatusString(ESetCharIntervalTimeout);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn int AbstractSerial::charIntervalTimeout() const
\public
\~russian Возвращает время ожидания прихода символа в приемный буфер последовательного устройства с которым сконфигурировано последовательное устройство.
\return \li Время ожидания прихода символа, в мсек \li 0 - в случае ошибки
\~english  Returns time of expectation of arrival of character in the receiving buffer of serial device which a serial device is configured with.
\return \li Time of expectation of arrival of character, in msek  \li 0 - in the case of error
*/
int AbstractSerial::charIntervalTimeout() const
{
	Q_D(const AbstractSerial);
	return (isValid()) ? d->serialEngine->charIntervalTimeout() : 0;
}

/*!
\fn bool AbstractSerial::setDtr(bool set)
\public
\~russian Устанавливает линию DTR в состояное "high" или "low" в зависимости от входного параметра
\param[in] set - желаемое состояние линии DTR
\return \li true - при успехе \li false - в случае ошибки
\~english Sets DTR line an state "high" or "low"
\param[in] set - desired state of line DTR
\return \li true - at success  \li false - at an error
*/
bool AbstractSerial::setDtr(bool set)
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->setDtr(set);
		if (!ret)
			emitStatusString(ESetDtr);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn bool AbstractSerial::setRts(bool set)
\public
\~russian Устанавливает линию RTS в состояное "high" или "low" в зависимости от входного параметра
\param[in] set - желаемое состояние линии RTS
\return \li true - при успехе \li false - в случае ошибки
\~english Sets DTR line an state "high" or "low"
\param[in] set - desired state of line RTS
\return \li true - at success  \li false - at an error
*/
bool AbstractSerial::setRts(bool set)
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->setRts(set);
		if (!ret)
			emitStatusString(ESetRts);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn ulong AbstractSerial::lineStatus()
\public
\~russian Возвращает статусы линий CTS, DSR, DCD, RI, RTS, DTR, ST, SR. Метод возвращает закодированные значения статусов линий побитовым ИЛИ.
\return \li true - при успехе \li false - в случае ошибки
\~english Returned the status lines CTS, DSR, DCD, RI, RTS, DTR, ST, SR. The method returns an encoded value status lines bitwise OR.
\return \li true - at success  \li false - at an error
*/
ulong AbstractSerial::lineStatus()
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->lineStatus();
		if (!ret)
			emitStatusString(ELineStatus);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn bool AbstractSerial::open(OpenMode mode)
\public
\~russian Открывает последовательное устройство.
\param[in] mode - режим открытия последовательного устройства
\return \li true - при успехе \li false - в случае ошибки
\note \li Последовательное устройство открывается с параметрами по умолчанию в режиме 9600 8 N 1 , и 50 мс время ожидания символа. После того, как устройство успешно открыто - можно приступать к его конфигурированию! \li Устройство правильно работает ТОЛЬКО при использовании режима QIODevice::Unbuffered
\~english Opens a serial device.
\param[in] mode - mode of opening of serial device
\return \li true - at success  \li false - in the case of error
\note \li  A serial device is opened with parameters by default in the mode 9600 8 N 1, and 50 ms time of expectation of character. Since a device is successful openly - it is possible to proceed to his configuring!  \li A device correctly works ONLY at the use of the mode of QIODevice::Unbuffered
*/
bool AbstractSerial::open(OpenMode mode)
{
	Q_D(AbstractSerial);

	if (!isOpen()) {
		if (isValid()) {
			if (d->serialEngine->open(mode)) {
				QIODevice::open(mode);

				if (QAbstractEventDispatcher::instance(thread()))
					d->setupSerialNotifiers();

				emitStatusString(ENoneOpen);
				return true;
			}
		}
		emitStatusString(EOpen);
		return false;
	}
	emitStatusString(EDeviceIsOpen);
	return false;
}

/*!
\fn void AbstractSerial::close()
\public
\~russian Закрывает последовательное устройство.
\~english Closes a serial device
*/
void AbstractSerial::close()
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		d->serialEngine->close();
		emitStatusString(ENoneClose);
	}
	else
		emitStatusString(EClose);
	QIODevice::close();
}

/*!
\fn bool AbstractSerial::flush()
\public
\~russian Очищает буферы последовательного устройства.
\return \li true - при успехе \li false - в случае ошибки
\~english Clears the buffers of serial device.
\return \li true - at success  \li false - at an error
*/
bool AbstractSerial::flush()
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->flush();
		if (!ret)
			emitStatusString(EFlush);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn bool AbstractSerial::reset()
\public
\~russian Сбрасывает буферы последовательного устройства.
\return \li true - при успехе \li false - в случае ошибки
\~english Reset the buffers of serial device.
\return \li true - at success  \li false - at an error
*/
bool AbstractSerial::reset()
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		bool ret = d->serialEngine->reset();
/*
		if (!ret)
			emitStatusString(EFlush);
*/
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return false;
}

/*!
\fn qint64 AbstractSerial::bytesAvailable(bool wait)
\public
\~russian Возвращает количество байт готовых для чтения, которые находятся во входном буфере последовательного устройства
\param wait - флаг ожидания прихода данных в течении времени charIntervalTimeout. Добавлен специально для корректной работы в ОС Windows NT/XP etc.
В ОС *.nix этот флаг просто не используется.
\return \li n - количество байт при успехе \li -1 - в случае ошибки
\~english A byte returns an amount ready for reading,
\return \li n - is an amount byte at success \li -1 - in the case of error
*/
qint64 AbstractSerial::bytesAvailable(bool wait)
{
	Q_D(AbstractSerial);
	if (isOpen()) {
		qint64 ret = d->serialEngine->bytesAvailable(wait);
		if (ret >= 0) {
		}
		else
			emitStatusString(EBytesAvailable);
		return ret;
	}
	emitStatusString(EDeviceIsNotOpen);
	return -1;
}

/*!
\fn bool AbstractSerial::isSequential() const
\public
\~russian
\~english
*/
bool AbstractSerial::isSequential() const
{
	return true;
}

/*
   Returns the difference between msecs and elapsed. If msecs is -1,
   however, -1 is returned.
*/
static int qt_timeout_value(int msecs, int elapsed)
{
	if (msecs == -1)
		return -1;
	int timeout = msecs - elapsed;
	return timeout < 0 ? 0 : timeout;
}

/*!
\fn bool AbstractSerial::waitForReadyRead(int msecs)
\public
\~russian Ожидает прихода во входной буфер последовательного устройства хотя-бы одного байта данных в течении определенного времени. При успешном выполнении излучается сигнал readyRead().
\param[in] msecs - время в течении которого ожидается приход символа, в мсек
\return \li true - при успехе \li false - в случае ошибки или при таймауте
\~english Expects arrival in the entrance buffer of serial device of even one byte of data in the flow of set time. The signal of readyRead() emitted at successful implementation().
\param[in] msecs - time in the flow of which is expected arrival of character, in msek
\return \li true - at success \li false - in the case of error or at a timeout
*/
bool AbstractSerial::waitForReadyRead(int msecs)
{
	Q_D(AbstractSerial);

	if (!isOpen()) {
		emitStatusString(EDeviceIsNotOpen);
		return false;
	}

	QTime stopWatch;
	stopWatch.start();

	forever {
		bool readyToRead = false;
		bool readyToWrite = false;
		if (!d->serialEngine->waitForReadOrWrite(&readyToRead, &readyToWrite, true, false,
													qt_timeout_value(msecs, stopWatch.elapsed()))) {
			emitStatusString(EWaitReadyReadTimeout);
			return false;
		}
		if (readyToRead) {
			if (bytesAvailable(false) > 0) {
				d->waitForReadyReadCalled = true;
				if (d->canReadNotification()) {
					return true;
				}
			}
		}
		if (readyToWrite)
			d->canWriteNotification();
	}
}

/*!
\fn bool AbstractSerial::waitForBytesWritten(int msecs)
\public
\~russian Ожидает ухода из выходного буфера последовательного устройства последнего символа в течении определенного времени.
\note В данной версии библиотеки этот метод пока не реализован.
\~english Expects a care from the output buffer of serial device of the last character in the flow of set time.
\note In this version of library this method while is not realized.
*/
bool AbstractSerial::waitForBytesWritten(int msecs)
{
	Q_UNUSED(msecs)
	return false;
}

/*!
\fn qint64 AbstractSerial::readData(char *data, qint64 maxSize)
\protected
\~russian Читает пакет данных из входного буфера последовательного устройства.
\param[out] *data - указатель на буфер (массив) в который происходит чтение пакета данных
\param[in] maxSize - количество байт, которые необходимо прочитать
\return \li n - количество реально прочитанных байт - при успехе \li -1 - в случае ошибки
\~english Reads the package of data from the input buffer of serial device.
\param[out] *data - a pointer to the buffer (array) in which takes a place reading of package of data
\param[in] maxSize - an amount is a byte, which must be read
\return \li n - an amount is real read byte - at success \li -1 - in the case of error
*/
qint64 AbstractSerial::readData(char *data, qint64 maxSize)
{
	Q_D(AbstractSerial);

	qint64 ret = d->serialEngine->read(data, maxSize);

	if (ret < 0)
		emitStatusString(EReadDataIO);

	d->emittedReadyRead = false;

	//qint64 bav = bytesAvailable();
	//Q_UNUSED(bav);
	return ret;
}

/*!
\fn qint64 AbstractSerial::writeData(const char *data, qint64 size)
\protected
\~russian Передает пакет данных в выходной буфер последовательного устройства. При успешном выполнении излучается сигнал bytesWritten().
\param[in] *data - указатель на буфер (массив) из которого происходит передача пакета данных
\param[in] maxSize - количество байт, которые необходимо передать
\return \li n - количество реально переданных байт - при успехе \li -1 - в случае ошибки
\~english Passes the package of data in the output buffer of serial device. The signal of bytesWritten() emitted at successful implementation.
\param[in] *data - a pointer to the buffer (array) from which takes a place write of package of data
\param[in] maxSize - an amount is a byte, which must be write
\return \li n - an amount is real write byte - at success  \li -1 - in the case of error
*/
qint64 AbstractSerial::writeData(const char *data, qint64 size)
{
	Q_D(AbstractSerial);

	qint64 ret = d->serialEngine->write(data, size);
	if (ret < 0)
		emitStatusString(EWriteDataIO);
	if (ret >= 0)
		emit bytesWritten(ret);

	return ret;
}

/*! Converted to baud rate value from string
*/
AbstractSerial::BaudRate AbstractSerial::baudRateFromString(const QString &str) const
{
	return baudRatesMap.key(str);
}

/*! Converted to data bits value from string
*/
AbstractSerial::DataBits AbstractSerial::dataBitsFromString(const QString &str) const
{
	return dataBitsMap.key(str);
}

/*! Converted to parity value from string
*/
AbstractSerial::Parity AbstractSerial::parityFromString(const QString &str) const
{
	return paritysMap.key(str);
}

/*! Converted to stop bits value from string
*/
AbstractSerial::StopBits AbstractSerial::stopBitsFromString(const QString &str) const
{
	return stopBitsMap.key(str);
}

/*! Converted to flow value from string
*/
AbstractSerial::Flow AbstractSerial::flowFromString(const QString &str) const
{
	return flowsMap.key(str);
}

/*! Converted to status value from string
*/
AbstractSerial::Status AbstractSerial::statusFromString(const QString &str) const
{
	return statusesMap.key(str);
}

/*! Converted from baud rate value to string
*/
QString AbstractSerial::baudRateToString(BaudRate val) const
{
	return baudRatesMap.value(val);
}

/*! Converted from data bits value to string
*/
QString AbstractSerial::dataBitsToString(DataBits val) const
{
	return dataBitsMap.value(val);
}

/*! Converted from parity value to string
*/
QString AbstractSerial::parityToString(Parity val) const
{
	return paritysMap.value(val);
}

/*! Converted from stop bits value to string
*/
QString AbstractSerial::stopBitsToString(StopBits val) const
{
	return stopBitsMap.value(val);
}

/*! Converted from flow value to string
*/
QString AbstractSerial::flowToString(Flow val) const
{
	return flowsMap.value(val);
}

/*! Converted from status value to string
*/
QString AbstractSerial::statusToString(Status val) const
{
	return statusesMap.value(val);
}

/*!
\fn void AbstractSerial::enableEmitStatus(bool enable)
\~russian Включает или отключает флаг испускания сигнала signalStatus()
\param[in] enable - желаемое значение флага
\note если: \li enable = true - то это значит, что включить испускание сигнала signalStatus() \li enable = false - то это значит, что отключить испускание сигнала signalStatus()
\~english On or Off the flag of emitting of signal of signalStatus()
\param[in] enable - desired value of flag
\note if: \li enable = true -  it means that to On emitting of signal of signalStatus()  \li enable = false -  it means that to Off emitting of signal of signalStatus()
*/
void AbstractSerial::enableEmitStatus(bool enable)
{
	Q_D(AbstractSerial);
	d->emittedStatus = enable;
}

/*! \internal
\~russian Возвращает значение флага, который определяет испускать или нет сигнал signalStatus()
\return Значение флага
\note если: \li true - то это значит, что испускание сигнала signalStatus() включено \li false - то это значит, что испускание сигнала signalStatus() отключено
\~english Returns the value of flag which determines to emit or not signal of signalStatus()
\note if: \li true - it means that emitting of signal of signalStatus() it is On \li false - it means that emitting of signal of signalStatus() it is Off.
*/
bool AbstractSerial::canEmitStatusString() const
{
	Q_D(const AbstractSerial);
	return d->emittedStatus;
}

/*! \internal
\~russian Испускает сигнал signalStatus()
\param[in] status - тип статуса \sa AbstractSerial::Status
\~english  Emits the signal of signalStatus()
\param[in] status - type of status \sa AbstractSerial::Status
*/
void AbstractSerial::emitStatusString(Status status)
{
	if (canEmitStatusString())
		emit signalStatus(statusToString(status), QDateTime::currentDateTime());
}

/*!
\fn AbstractSerial::signalStatus(const QString &status, QDateTime current)
\~russian Этот сигнал испускается всякий раз при изменении статуса объекта  AbstractSerial при вызове его методов из секций public и protected.
Изменение статуса может быть вызвано следующими причинами: \li при ошибках конфигурировании устройства \li при его открытии/закрытии \li при ошибках чтения/записи данных и т.п.
\param[out] status - текущий статус последовательного устройства
\param[out] current - текущее значение даты/времени произошедшей смены статуса
\~english This signal is emitted every time at the change of status of object  of Abstractserial at the call of his methods from the sections of public and protected. The change of status can be caused the followings reasons: \li at errors configuring of device \li at his opening/closing \li at the errors of reading/writting of data etc.
\note \~russian Применять этот сигнал можно в GUI приложении например таким образом: \~english Applying this signal is possible in GUI appendix for example thus:
\~
\code
...
    connect( this, SIGNAL(signalStatus(const QString &, QDateTime)), widget, SLOT(update(const QString &, QDateTime)) );
...
...
void wigdet::update(const QString &msg, QDateTime dt)
{
...
    QString dtString = dt.time().toString();
    QString str = msg + ", in time:" + dtString;
...
    listWidget->addItem(str);
...
}
\endcode
*/
