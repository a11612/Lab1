#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <QCoreApplication>
#include <QTimer>
#include <QObject>

#include "qmqtt/qmqtt.h"

class Subscriber : public QMQTT::Client
{
    Q_OBJECT
public:
    explicit Subscriber(const QString& hostname = "",
                        const quint16 port = 0,
                        QObject* parent = nullptr)
        //: QMQTT::Client(host, port, parent)
        : QMQTT::Client(hostname, port, false, true, parent)
        , _qout(stdout)
    {
        _topic = "";
        connect(this, &Subscriber::connected, this, &Subscriber::onConnected);
        connect(this, &Subscriber::subscribed, this, &Subscriber::onSubscribed);
        connect(this, &Subscriber::received, this, &Subscriber::onReceived);
    }
    virtual ~Subscriber() {}

    QTextStream _qout;

public slots:
    void setTopic(const QString& topic)
    {
        _topic = topic;
    }

    void onConnected()
    {
        _qout << "subscriber connected" << endl;
        subscribe(_topic, 0);
    }

    void onSubscribed(const QString& topic)
    {
        _qout << "subscribed to: " << topic << endl;
    }

    void onReceived(const QMQTT::Message& message)
    {
        _qout << "publish received: \""
              << QString::fromUtf8(message.payload())
              << "\"" << endl;
    }

private:
    QString _topic;

};

#endif // SUBSCRIBER_H
