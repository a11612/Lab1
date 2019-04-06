#include <QGuiApplication>
//#include <QApplication>
#include <QQmlApplicationEngine>

#include "qmqtt/qmqtt.h"
#include "mainwindow.h"
#include "subscriber.h"
#include "publisher.h"

#include "backend.h"
#include <QQmlContext>
#include "wiringPi.h"



const QString EXAMPLE_HOST = "test.mosquitto.org";
const quint16 EXAMPLE_PORT = 1883;
const QString EXAMPLE_TOPIC = "qmqtt/AnaMichell";

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/main.qml")));
    if (engine.rootObjects().isEmpty())
        return -1;
    BackEnd back(nullptr, &engine);


    engine.rootContext()->setContextProperty("BackEnd", &back);


    return app.exec();

}
