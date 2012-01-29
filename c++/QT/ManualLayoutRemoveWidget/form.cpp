#include "form.h"

#include <QPushButton>
#include <QCheckBox>
#include <QVBoxLayout>

#include <iostream>

MyForm::MyForm(QWidget *parent) : QWidget(parent)
{
  QPushButton* btnAdd = new QPushButton("Add", this );
  connect(btnAdd, SIGNAL(clicked()), this, SLOT(Add()));
  
  QPushButton* btnRemove = new QPushButton("Remove", this );
  connect(btnRemove, SIGNAL(clicked()), this, SLOT(Remove()));

  this->Layout = new QVBoxLayout(this );
  //this->setSizePolicy();
  this->Layout->addWidget(btnAdd);
  this->Layout->addWidget(btnRemove);

  this->setLayout(this->Layout);
  this->show();

}

void MyForm::Add()
{
  QString checkBoxName = QString::number(this->CheckBoxes.size());
  QCheckBox* checkBox = new QCheckBox(checkBoxName, this);
  this->CheckBoxes.push_back(checkBox);
  this->Layout->addWidget(checkBox);
  std::cout << "Added " << checkBoxName.toStdString() << std::endl;
}

void MyForm::Remove()
{
  this->Layout->removeWidget(this->CheckBoxes[this->CheckBoxes.size() - 1]);
  delete this->CheckBoxes[this->CheckBoxes.size() - 1];
  
  this->CheckBoxes.resize(this->CheckBoxes.size() - 1);
  
}
