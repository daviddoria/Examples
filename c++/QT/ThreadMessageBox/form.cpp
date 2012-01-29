#include <QMessageBox>

#include "form.h"
#include "thread.h"

Form::Form(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
  connect( this->pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_clicked()) );
  connect( &(this->MyThread), SIGNAL( DisplayMessageBoxSignal() ), this, SLOT(DisplayMessageBoxSlot()) );
}

void Form::pushButton_clicked()
{
  MyThread.start();
}

void Form::DisplayMessageBoxSlot()
{
  QMessageBox msgBox;
  msgBox.setText("Test Text");
  msgBox.exec();
}