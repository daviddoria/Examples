#include "dialog.h"

Dialog::Dialog()
{
  setupUi(this);
}

void Dialog::on_buttonBox_accepted()
{
  this->setResult(QDialog::Accepted);
  this->userText = this->lineEdit->text().toStdString();
}

void Dialog::on_buttonBox_rejected()
{
  this->setResult(QDialog::Rejected);
}

std::string Dialog::GetUserText()
{
  return this->userText;
}
