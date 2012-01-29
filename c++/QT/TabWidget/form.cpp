#include <QtGui>

#include "form.h"

#include <QTableWidgetItem>

Form::Form(QWidget *parent)  : QMainWindow(parent)
{
  setupUi(this);
  this->pushButton->setText("Hello");
}
