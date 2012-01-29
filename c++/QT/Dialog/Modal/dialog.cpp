#include "dialog.h"

Dialog::Dialog()
{
  setupUi(this);
}

void Dialog::on_buttonBox_accepted()
{
  this->setResult(QDialog::Accepted);
  //this->accept();
}

void Dialog::on_buttonBox_rejected()
{
  this->setResult(QDialog::Rejected);
  //this->reject();
}
