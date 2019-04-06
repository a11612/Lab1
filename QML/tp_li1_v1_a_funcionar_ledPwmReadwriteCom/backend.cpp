#include "backend.h"
#include "wiringPi.h"

BackEnd::BackEnd(QObject *parent, QQmlApplicationEngine *engine) : QObject (parent)
{

    eng = engine;
    wiringPiSetup();
    pinMode(0, OUTPUT);
    //pinMode(1,PWM_OUTPUT);

    //pwmWrite(1,0);

    _com = new QSerialPort();
    _com->setPortName("/dev/ttyACM0");
    _com->setBaudRate(115200);
    _com->setDataBits(QSerialPort::DataBits::Data8);
    _com->setParity(QSerialPort::Parity::NoParity);
    _com->setStopBits(QSerialPort::StopBits::TwoStop);
    _com->setFlowControl(QSerialPort::FlowControl::NoFlowControl);





    connect(_com, &QSerialPort::readyRead, this, &BackEnd::onComReceive);

    if(_com->open(QIODevice::ReadWrite))
    {
        qDebug()<<"Port COM open";
    }
    else
    {
        qDebug()<<"Port COM closed";
    }






}


void BackEnd::onPinOn()
{
    digitalWrite(5, HIGH);

}

void BackEnd::onPinOff()
{
    digitalWrite(0, LOW);

}

void BackEnd::valueSlider(int value)
{
    qDebug()<< value;
    //int val=value;

    //pwmWrite(1, value);
    
}



void BackEnd::onComReceive()
{
    //qDebug()<<"new data";

    if(_com->canReadLine())
    {

        QString data= _com->readLine();
         //qDebug()<< data;

         if (stackIndex == 2)
         {
             _textBox1->setProperty("text", data);
         }





         //QString data= QString(_com->readLine());
         // QStringList list== data.split('\t');
         //qDebug<< list.at(0);
    }
}

void BackEnd::onPageChange(QString title, int index)
{
    stackIndex = index;
    qDebug() << stackIndex;
    if (title == "Light")
    {
        _textBox1 = eng->rootObjects().at(0)->findChild<QObject*>("textBox1");

    }
}

void BackEnd::tempChange(int temp)
{
    qDebug() << temp;
    QString data =QString::number(temp);
    data = data + "\n";
    QByteArray sendData = data.toUtf8();
    _com->write(sendData);
}


