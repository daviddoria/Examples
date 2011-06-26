#include "form.h"

#include <QPushButton>

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  setupUi(this);

  QPushButton* hello = new QPushButton( "Test", this );
  hello->show();

//  connect( this->pushButton, SIGNAL( clicked() ), this, SLOT(pushButton_SetLabelText()) );
}

void MyForm::pushButton_SetLabelText()
{
  //this->label->setText("hello");
}
