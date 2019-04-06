

    #include "lcdnumber.h"

    Lcdnumber :: Lcdnumber(QGraphicsItem *parent) : QGraphicsProxyWidget(parent)
    {
      number = new QLCDNumber();
      number->display(11);
    }

    int main (int argc, char *argv[])
    {
      QApplication app (argc, argv);

      qmlRegisterType <Lcdnumber> ("Q", 1, 0, "Lcdnumber");

      QDeclarativeView view;
      view.setSource (QUrl :: fromLocalFile ("qq.qml"));
      view.show ();

      return app.exec();
    }
