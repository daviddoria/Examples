#include <QtGui>
#include <iostream>

#include "form.h"
#include "dialog.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
}


void Form::on_pushButton_clicked()
{
  this->label->setText(QString::number(this->horizontalSlider->value()));
  std::cout << this->label->text().toDouble() << std::endl;

  // Modal - Using this, the user can only interacting with the dialog until it is closed.
  //Dialog* myDialog(new Dialog);
  //myDialog->exec();

  // Need to store myDialog as a member!
  // Modeless
  if (!myDialog)
  {
    myDialog = new FindDialog(this);
    //connect(findDialog, SIGNAL(findNext()), this, SLOT(findNext()));
  }
  myDialog->show();
  myDialog->raise();
  myDialog->activateWindow();
}
