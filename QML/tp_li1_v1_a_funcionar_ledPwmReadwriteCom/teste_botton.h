#ifndef TESTE_BOTTON_H
#define TESTE_BOTTON_H

#include <QDialog>

namespace Ui {
class teste_botton;
}

class teste_botton : public QDialog
{
    Q_OBJECT

public:
    explicit teste_botton(QWidget *parent = nullptr);
    ~teste_botton();

private:
    Ui::teste_botton *ui;
};

#endif // TESTE_BOTTON_H
