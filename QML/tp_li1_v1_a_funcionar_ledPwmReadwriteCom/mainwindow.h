#ifndef MAINWINDOW_H
#define MAINWINDOW_H


 // #include <QSerialPort>
#include "qmqtt/qmqtt.h"
#include <QObject>


/*namespace Ui {
class MainWindow;
}*/

class MainWindow : public QObject
{
    Q_OBJECT

public:
    explicit MainWindow(QObject *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_Received(const QMQTT::Message& message);
    void on_Connected();
    void on_Subscribed(const QString& topic);
    void on_COM_Receive();

private:
   // Ui::MainWindow *ui;
    QMQTT::Client *_mqtt;
    //QSerialPort *_com;
};

#endif // MAINWINDOW_H
