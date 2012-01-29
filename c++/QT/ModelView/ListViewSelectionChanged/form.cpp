#include <QtGui>
#include <QStringListModel>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent)
    : QMainWindow(parent)
{
  setupUi(this);
  std::cout << "Current path: " << QDir::currentPath().toStdString() << std::endl;
  
  this->model = new QStringListModel;
  
  this->listView->setModel(model);

  this->lblPath->setText(QDir::currentPath());
  this->lblFile->setText(listView->currentIndex().data(QFileSystemModel::FilePathRole).toString());

}


void Form::on_listView_clicked(const QModelIndex & index)
{
  QModelIndexList indexList = this->listView->selectedIndexes();
}
