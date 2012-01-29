#include <QtGui>
#include <QButtonGroup>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent)
    : QWidget(parent)
{
  this->setupUi(this);

  QButtonGroup* group1 = new QButtonGroup;
  group1->addButton(rad1);
  group1->addButton(rad2);
  
  QButtonGroup* group2 = new QButtonGroup;
  group2->addButton(radA);
  group2->addButton(radB);
}

void Form::on_rad1_clicked()
{
  std::cout << "1" << std::endl;
}

void Form::on_rad2_clicked()
{
  std::cout << "2" << std::endl;
}

void Form::on_radA_clicked()
{
  std::cout << "A" << std::endl;
}

void Form::on_radB_clicked()
{
  std::cout << "B" << std::endl;
}

