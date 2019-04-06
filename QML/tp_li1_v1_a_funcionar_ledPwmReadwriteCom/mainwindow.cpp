#include "mainwindow.h"


const QString EXAMPLE_HOST = "test.mosquitto.org";
const quint16 EXAMPLE_PORT = 1883;
const QString EXAMPLE_TOPIC = "qmqtt/AnaMichell";

MainWindow::MainWindow(QObject *parent):QObject(parent)
{

/*
    //connect COM Port
    _com= new QSerialPort();
    _com->setPortName("/dev/ttyACM0"); //confirmar isto
    _com->setBaudRate(115200);
    _com->setDataBits(QSerialPort::DataBits::Data8);
    _com->setParity(QSerialPort::Parity::NoParity);
    _com->setStopBits(QSerialPort::StopBits::TwoStop);
    _com->setFlowControl(QSerialPort::FlowControl::NoFlowControl);


    connect(_com, &QSerialPort::readyRead, this, &MainWindow::on_COM_Receive);

    if(_com->open(QIODevice::ReadWrite))
    {
        qDebug()<<"Port COM open";
    }
    else
    {
        qDebug()<<"Port COM closed";
    }

    _mqtt = new QMQTT::Client(EXAMPLE_HOST, EXAMPLE_PORT, false, true);

    connect(_mqtt, &QMQTT::Client::received, this, &MainWindow::on_Received);
    connect(_mqtt, &QMQTT::Client::connected, this, &MainWindow::on_Connected);
    connect(_mqtt, &QMQTT::Client::subscribed, this, &MainWindow::on_Subscribed);

    _mqtt->connectToHost();
}

MainWindow::~MainWindow()
{

}

void MainWindow::on_pushButton_clicked()
{
    if(_mqtt->isConnectedToHost())
    {
        //QMQTT::Message message(0, EXAMPLE_TOPIC, ui->lineEdit_toSend->text().toUtf8());
        //_mqtt->publish(message);
    }
    else
        qDebug() << "MQTT not connected!";
}

void MainWindow::on_Received(const QMQTT::Message& message)
{
    qDebug() << "on_Received: "
             << QString::fromUtf8(message.payload());

   // ui->lineEdit_received->setText(message.payload());
}

void MainWindow::on_Connected()
{
    qDebug() << "on_Connected ";
    _mqtt->subscribe(EXAMPLE_TOPIC, 0); //QoS = 0
}

void MainWindow::on_Subscribed(const QString& topic)
{
    qDebug() << "on_Subscribed " << topic;
}

void MainWindow::on_COM_Receive()
{
    qDebug()<<"new data";

    if(_com->canReadLine())
    {
         qDebug()<<_com->readLine();

         //QString data= QString(_com->readLine());
         // QStringList list== data.split('\t');
         //qDebug<< list.at(0);
    }
    */
}
