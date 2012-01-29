#include "form.h"

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

}

void MyForm::on_buttonBox_accepted()
{
  this->label->setText("accepted");
}

void MyForm::on_buttonBox_rejected()
{
  this->label->setText("rejected");
}
