#
PROJECT         = Serial Device Liblary
TEMPLATE        = lib

CONFIG          += staticlib
CONFIG          -= debug_and_release debug
QT              -= gui

OBJECTS_DIR     = build/obj
MOC_DIR         = build/moc
RCC_DIR         += build/rcc
#RESOURCES      += resources.qrc
#TRANSLATIONS   += translations/qserialdevice_ru.ts
DEPENDDIR       = .
INCLUDEDIR      = .

HEADERS         = datatypes.h            \
                  abstractserial.h       \
                  abstractserialengine.h \
                  nativeserialengine.h

SOURCES         = abstractserial.cpp       \
                  abstractserialengine.cpp \
                  nativeserialengine.cpp


win32:HEADERS   += winserialnotifier.h
win32:SOURCES   += nativeserialengine_win.cpp \
                   winserialnotifier.cpp

unix:SOURCES    += nativeserialengine_unix.cpp


DESTDIR         = release
TARGET          = qserialdevice


VERSION         = 0.1.0
