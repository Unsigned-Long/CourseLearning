QT       += core gui charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    angle.h \
    handler.h \
    mainwindow.h \
    point.hpp \
    utility.hpp

FORMS += \
    mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

unix:!macx: LIBS += -L$$PWD/../../build/ -lcurveHandler

INCLUDEPATH += $$PWD/../../build
DEPENDPATH += $$PWD/../../build

unix:!macx: LIBS += -L$$PWD/../../thirdparty/libs/ -langle

INCLUDEPATH += $$PWD/../../thirdparty/libs
DEPENDPATH += $$PWD/../../thirdparty/libs
