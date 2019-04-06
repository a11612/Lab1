#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <QCoreApplication>
#include <QTimer>
#include <QObject>

#include "qmqtt/qmqtt.h"

class Publisher : public QMQTT::Client
{
    Q_OBJECT
public:
    explicit Publisher(const QString& hostname = "",
                       const quint16 port = 0,
                       QObject* parent = nullptr)
        : QMQTT::Client(hostname, port, false, true, parent)
        , _number(0)
        , _qout(stdout)
    {
        _topic = "";
        connect(this, &Publisher::connected, this, &Publisher::onConnected);
        connect(&_timer, &QTimer::timeout, this, &Publisher::onTimeout);
        connect(this, &Publisher::disconnected, this, &Publisher::onDisconnected);
    }
    virtual ~Publisher() {}

    QTimer _timer;
    quint16 _number;
    QTextStream _qout;

public slots:
    void setTopic(const QString& topic)
    {
        _topic = topic;
    }

    void onConnected()
    {
        _qout << "publisher connected" << endl;

        subscribe(_topic, 0);
        _timer.start(1000);
    }

    void onTimeout()
    {
        _qout << "publisher timeout" << endl;

        QMQTT::Message message(_number, _topic,
                               QString("Number is %1").arg(_number).toUtf8());
        publish(message);
        _number++;
        if(_number >= 10)
        {
            _timer.stop();
            disconnectFromHost();
        }
    }

    void onDisconnected()
    {
        _qout << "publisher disconnected" << endl;

        QTimer::singleShot(0, qApp, &QCoreApplication::quit);
    }

private:
    QString _topic;
};

#endif // PUBLISHER_H
