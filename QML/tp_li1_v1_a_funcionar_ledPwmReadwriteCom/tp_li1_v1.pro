QT += quick serialport network
CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        main.cpp \
    qmqtt/qmqtt_websocketiodevice.cpp \
    qmqtt/qmqtt_websocket.cpp \
    qmqtt/qmqtt_timer.cpp \
    qmqtt/qmqtt_ssl_socket.cpp \
    qmqtt/qmqtt_socket.cpp \
    qmqtt/qmqtt_routesubscription.cpp \
    qmqtt/qmqtt_router.cpp \
    qmqtt/qmqtt_network.cpp \
    qmqtt/qmqtt_message.cpp \
    qmqtt/qmqtt_frame.cpp \
    qmqtt/qmqtt_client_p.cpp \
    qmqtt/qmqtt_client.cpp \
    backend.cpp

RESOURCES += \
    qml.qrc

LIBS +=-L//home/michell/raspi/sysroot/usr/lib -lwiringPi

# Additional import path used to resolve QML modules in Qt Creator's code model
QML_IMPORT_PATH =

# Additional import path used to resolve QML modules just for Qt Quick Designer
QML_DESIGNER_IMPORT_PATH =

# Default rules for deployment.
#qnx: target.path = /tmp/$${TARGET}/bin
#else: unix:!android: target.path = /opt/$${TARGET}/bin
#!isEmpty(target.path): INSTALLS += target

  target.files = tp_li1_v1
  target.path = /home/pi
  INSTALLS += target

HEADERS += \
    subscriber.h \
    publisher.h \
    subscriber.h \
    qmqtt/qmqtt_websocket_p.h \
    qmqtt/qmqtt_websocketiodevice_p.h \
    qmqtt/qmqtt_timer_p.h \
    qmqtt/qmqtt_timerinterface.h \
    qmqtt/qmqtt_ssl_socket_p.h \
    qmqtt/qmqtt_socket_p.h \
    qmqtt/qmqtt_socketinterface.h \
    qmqtt/qmqtt_routesubscription.h \
    qmqtt/qmqtt_router.h \
    qmqtt/qmqtt_routedmessage.h \
    qmqtt/qmqtt_network_p.h \
    qmqtt/qmqtt_networkinterface.h \
    qmqtt/qmqtt_message_p.h \
    qmqtt/qmqtt_message.h \
    qmqtt/qmqtt_global.h \
    qmqtt/qmqtt_frame.h \
    qmqtt/qmqtt_client_p.h \
    qmqtt/qmqtt_client.h \
    qmqtt/qmqtt.h \
    backend.h \
    lcdnumber.h

DISTFILES +=

FORMS +=
