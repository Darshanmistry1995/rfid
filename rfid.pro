#-------------------------------------------------
#
# Project created by QtCreator 2016-09-22T12:54:15
#
#-------------------------------------------------

QT       += core
QT       += network
QT       -= gui

TARGET = rfid
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    qserialdevice/src/nativeserialengine_unix.cpp \
    qserialdevice/src/nativeserialengine.cpp \
    qserialdevice/src/abstractserialengine.cpp \
    qserialdevice/src/abstractserial.cpp \
    worker.cpp \
    mfrc522.cpp

HEADERS += \
    qserialdevice/src/nativeserialengine.h \
    qserialdevice/src/datatypes.h \
    qserialdevice/src/abstractserialengine.h \
    qserialdevice/src/abstractserial.h \
    worker.h \
    mfrc522.h

# За QSerialDevice
DEPENDPATH += .
INCLUDEPATH += ./qserialdevice/src
INCLUDEPATH += ./mfrc522
QMAKE_LIBDIR += ./qserialdevice/src/release
LIBS += -lqserialdevice
LIBS += -L"$$PWD/qserialdevice/src/release"

OTHER_FILES += \
    setEnv.sh
