#-------------------------------------------------
#
# Project created by QtCreator 2016-07-12T13:37:15
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = FishSwarm
TEMPLATE = app


SOURCES += main.cpp\
    lures.cpp \
    fishrobot.cpp \
    swarminterface.cpp \
    wallfollowing.cpp

HEADERS  += \
    lures.h \
    fishrobot.h \
    swarminterface.h \
    constants.h \
    wallfollowing.h

FORMS    += \
    swarminterface.ui \

RESOURCES += \
    fishressources.qrc
