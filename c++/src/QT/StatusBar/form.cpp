#include <QtGui>

#include "form.h"

Form::Form(QWidget *parent)
    : QMainWindow(parent)
{
  setupUi(this);
}

void Form::on_pushButton_clicked()
{
  this->statusBar()->showMessage("Set label text.");
}
