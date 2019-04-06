#ifndef LCD_H
#define LCD_H




#include <QtDeclarative/QDeclarativeExtensionPlugin>
#include <QtDeclarative/qdeclarative.h>
#include <QGraphicsProxyWidget>
#include <QDebug>

#include <qdeclarative.h>
#include <QDeclarativeView>
#include <QApplication>

#include <QWidget>
#include <QLCDNumber>


class Lcdnumber : public QGraphicsProxyWidget
{
    Q_OBJECT

public:
  Lcdnumber(QGraphicsItem *parent = 0);

  ~Lcdnumber() {}

private:
    QLCDNumber* number;
};

#endif // LCDNUMBER_H

#endif // LCD_H
