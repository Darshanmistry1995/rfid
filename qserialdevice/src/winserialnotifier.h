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

#ifndef WINSERIALNOTIFIER_H
#define WINSERIALNOTIFIER_H

#include <qt_windows.h>
#include <QObject>

class QWinEventNotifier;
class WinSerialNotifier : public QObject
{
	Q_OBJECT
public:
	enum Type { Read, Write, Exception };

	WinSerialNotifier(HANDLE hd, Type type, QObject *parent = 0);
	~WinSerialNotifier();

	inline HANDLE hd() const { return shd; }
	inline Type type() const { return wsntype; }

	inline bool isEnabled() const { return wsnenabled; }

public Q_SLOTS:
	void setEnabled(bool);

Q_SIGNALS:
	void activated(HANDLE hd);

private slots:
	void updateNotifier(HANDLE h);

private:
	Q_DISABLE_COPY(WinSerialNotifier)

	HANDLE shd;
	Type wsntype;
	bool wsnenabled;
	QWinEventNotifier *wen;
	OVERLAPPED ovl;
	DWORD eventMask;
};

#endif // WINSERIALNOTIFIER_H
