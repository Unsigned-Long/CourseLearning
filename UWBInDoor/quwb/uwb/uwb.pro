QT       += core gui charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

ICON += uwb.png

SOURCES += \
    helper.cpp \
    main.cpp \
    uwbwin.cpp

HEADERS += \
    UWBContext.h \
    ceresSolver.h \
    datatime.h \
    helper.h \
    linear.h \
    newtonLS.h \
    point.h \
    sequential.h \
    taylorSeries.h \
    timer.h \
    uwbwin.h

FORMS += \
    helper.ui \
    uwbwin.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

unix:!macx: LIBS += -L$$PWD/../../lib/ -luwb

INCLUDEPATH += $$PWD/../../include
DEPENDPATH += $$PWD/../../include

INCLUDEPATH += /usr/local/include/eigen3

unix:!macx: LIBS += -L$$PWD/../../thirdparty/lib/ -ldatetime

INCLUDEPATH += $$PWD/../../thirdparty/include
DEPENDPATH += $$PWD/../../thirdparty/include
