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

#ifndef DATATYPES_H
#define DATATYPES_H

/*! 
\~english Depending on the platform to determine the names of serial devices used in the OS
\~russian В зависимости от платформы определяем имена последовательных устройств используемых в ОС
*/
#ifdef Q_OS_WIN32
#define DEFAULT_DEVICE_NAME     "COM1"
#elif defined(Q_OS_IRIX)
#define DEFAULT_DEVICE_NAME     "/dev/ttyf1"
#elif defined(Q_OS_HPUX)
#define DEFAULT_DEVICE_NAME     "/dev/tty1p0"
#elif defined(Q_OS_SOLARIS)
#define DEFAULT_DEVICE_NAME     "/dev/ttya"
#elif defined(Q_OS_FREEBSD)
#define DEFAULT_DEVICE_NAME     "/dev/ttyd1"
#elif defined(Q_OS_LINUX)
#define DEFAULT_DEVICE_NAME     "/dev/ttyS0"
#else
#define DEFAULT_DEVICE_NAME     "/dev/ttyS0"
#endif

/*!
\~english Constants status line interface RS232.
\~russian Константы статусов линий интерфейса RS232
*/
#define LS_CTS  0x01
#define LS_DSR  0x02
#define LS_DCD  0x04
#define LS_RI   0x08
#define LS_RTS  0x10
#define LS_DTR  0x20
#define LS_ST   0x40
#define LS_SR   0x80

#endif
