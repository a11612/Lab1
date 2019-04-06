#ifndef BACKEND_H
#define BACKEND_H

#include <QObject>

#include <QQmlApplicationEngine>

#include "wiringPi.h"

#include <qfilesystemwatcher.h>
#include <QFile>
#include <QDebug>
#include <QtSerialPort>

class BackEnd: public QObject
{

    Q_OBJECT


public:
    explicit BackEnd(QObject *parent = nullptr, QQmlApplicationEngine *engine = nullptr);

public slots:

    void onPinOn();
    void onPinOff();
    void valueSlider(int value);

    void onComReceive();

    void onPageChange(QString title, int index);
    void tempChange(int temp);


private:

    QSerialPort *_com;

    QObject *_textBox1;


    QQmlApplicationEngine *eng;

    int stackIndex =0;
};

#endif // BACKEND_H
