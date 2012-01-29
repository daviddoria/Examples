#include <QtGui>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent)
    : QMainWindow(parent)
{
  setupUi(this);
  
  QFileSystemModel *model = new QFileSystemModel;
  model->setRootPath(QDir::currentPath());
  treeView->setModel(model);
  //treeView->setRootIndex(model->index(QDir::currentPath()));
  treeView->setCurrentIndex(model->index(QDir::currentPath()));
  
}

void Form::on_treeView_clicked()
{
  std::cout << treeView->currentIndex().data(QFileSystemModel::FilePathRole).toString().toStdString() << std::endl;
  std::cout << treeView->currentIndex().data(QFileSystemModel::FileNameRole).toString().toStdString() << std::endl;
}
