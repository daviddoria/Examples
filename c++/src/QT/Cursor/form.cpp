#include "form.h"

#include <QCursor>

MyForm::MyForm(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  connect( this->pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_clicked()) );
  this->isWaiting = false;
}

void MyForm::pushButton_clicked()
{
  this->label->setText("hello");

  QCursor waitCursor;
  if(!isWaiting)
    {
    waitCursor.setShape(Qt::WaitCursor);
    isWaiting = true;
    }
  else
    {
    waitCursor.setShape(Qt::ArrowCursor);
    isWaiting = false;
    }
  this->setCursor(waitCursor);
}
