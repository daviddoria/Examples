#include <QtGui>

#include "form.h"

#include "FileSelectionWidget.h"

#include <iostream>
#include <string>

#include <QFileDialog>
#include <QString>

Form::Form(QWidget* parent)
    : QWidget(parent)
{
  setupUi(this);


  FileSelectionWidget* Widget1 = new FileSelectionWidget(this );
  this->horizontalLayout->addWidget(Widget1);

  FileSelectionWidget* Widget2 = new FileSelectionWidget(this );
  this->horizontalLayout->addWidget(Widget2);

  
  Widget1->Test();

}
