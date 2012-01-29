#include "form.h"

#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>

MyForm::MyForm(QWidget *parent)
    : QWidget(parent)
{
  // Simple
//   QPushButton* btnHello = new QPushButton( "Hello", this );
//   QPushButton* btnHello2 = new QPushButton( "Hello2", this );
// 
//   QHBoxLayout* layout = new QHBoxLayout(this );
//   layout->addWidget(btnHello);
//   layout->addWidget(btnHello2);
// 
//   this->setLayout(layout);
//   this->show();

  // Advanced
  QPushButton* btnHello = new QPushButton( "Hello", this );
  QPushButton* btnHello2 = new QPushButton( "Hello2", this );
  QPushButton* btnHello3 = new QPushButton( "Hello3", this );

  QHBoxLayout* HLayout = new QHBoxLayout;
  HLayout->addWidget(btnHello);
  HLayout->addWidget(btnHello2);

  QVBoxLayout* VLayout = new QVBoxLayout;
  VLayout->addLayout(HLayout);
  VLayout->addWidget(btnHello3);
  
  this->setLayout(VLayout);
  this->show();

}
