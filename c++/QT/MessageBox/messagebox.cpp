#include "messagebox.h"

#include <QMessageBox>

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);
  connect( this->btnClickMe, SIGNAL( clicked() ), this, SLOT(btnClickMe_clicked()) );
}

void MyForm::btnClickMe_clicked()
{
  QMessageBox msgBox;
  msgBox.setText("Test Text");
  msgBox.exec();
}
