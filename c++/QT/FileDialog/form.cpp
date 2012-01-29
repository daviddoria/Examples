#include <QtGui>

#include "form.h"

#include <iostream>
#include <string>

#include <QFileDialog>
#include <QString>

Form::Form(QWidget* parent)
    : QWidget(parent)
{
    setupUi(this);

    connect( this->btnSave, SIGNAL( clicked() ), this, SLOT(btnSave_clicked()) );
    connect( this->btnOpenFile, SIGNAL( clicked() ), this, SLOT(btnOpenFile_clicked()) );
    connect( this->btnOpenDirectory, SIGNAL( clicked() ), this, SLOT(btnOpenDirectory_clicked()) );
}

void Form::btnOpenFile_clicked()
{
  //get a filename to open
  QString fileName = QFileDialog::getOpenFileName(this,
                    "OpenFile", "/home/doriad", "All Files (*.*)");
     //tr("Open Image"), "/home/doriad", tr("Image Files (*.png *.jpg *.bmp)"));

  std::cout << "Got filename: " << fileName.toStdString() << std::endl;

}

void Form::btnOpenDirectory_clicked()
{
  //get a filename to open
  QString directoryName = QFileDialog::getExistingDirectory(this,
     "Open Directory", "/home/doriad", QFileDialog::ShowDirsOnly);

  std::cout << "Got filename: " << directoryName.toStdString() << std::endl;
}

void Form::btnSave_clicked()
{

  //set a filename to save
  QString fileName = QFileDialog::getSaveFileName(this,
     tr("Save Image"), "/home/doriad", tr("Image Files (*.png *.jpg *.bmp)"));

  std::cout << "Set filename: " << fileName.toStdString() << std::endl;
}