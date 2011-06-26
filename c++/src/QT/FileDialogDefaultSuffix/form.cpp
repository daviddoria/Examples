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
}

void Form::on_btnSave_clicked()
{
  /*
  // This doesn't work because it still calls the static getSaveFileName
  QFileDialog dialog(this);
  //dialog.setFilter(".png .bmp");
  dialog.setNameFilter("*.png;;*.bmp");
  dialog.setDefaultSuffix("png");
  QString fileName = dialog.getSaveFileName();
  */

  QFileDialog dialog(this);
  dialog.setNameFilter("*.png;;*.bmp");
  dialog.setDefaultSuffix("png");
  
  dialog.exec();
  
  QString fileName = dialog.selectedFiles()[0];
  
  std::cout << "Set save filename: " << fileName.toStdString() << std::endl;
}
