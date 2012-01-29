#include <QtGui>
#include <iostream>
#include <sstream>

#include "form.h"
#include "dialog.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  this->ModelessDialog = new Dialog;
  connect(this->ModelessDialog, SIGNAL(accepted()), this, SLOT(ModelessDialog_accepted()));
  connect(this->ModelessDialog, SIGNAL(rejected()), this, SLOT(ModelessDialog_rejected()));
}

void Form::ModelessDialog_accepted()
{
  int result = this->ModelessDialog->result();
  std::stringstream ss;
  ss << result;
  this->lblModelessResult->setText(ss.str().c_str());
}

void Form::ModelessDialog_rejected()
{
  int result = this->ModelessDialog->result();
  std::stringstream ss;
  ss << result;
  this->lblModelessResult->setText(ss.str().c_str());
}

void Form::on_btnShowModeless_clicked()
{
  ModelessDialog->show();
  ModelessDialog->raise();
  ModelessDialog->activateWindow();
}
