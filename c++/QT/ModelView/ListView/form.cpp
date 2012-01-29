#include <QtGui>

#include <iostream>

#include "form.h"

Form::Form(QWidget *parent)
    : QMainWindow(parent)
{
  setupUi(this);
  std::cout << "Current path: " << QDir::currentPath().toStdString() << std::endl;
  
  this->model = new QFileSystemModel;
  this->model->setRootPath(QDir::currentPath());
  this->listView->setModel(model);
  this->listView->setRootIndex(model->index(QDir::currentPath()));

  this->lblPath->setText(QDir::currentPath());
  this->lblFile->setText(listView->currentIndex().data(QFileSystemModel::FilePathRole).toString());
  //this->listView->setCurrentIndex(model->index(QDir::currentPath()));
  
}

void Form::on_listView_doubleClicked(const QModelIndex & index)
{
  std::cout << listView->currentIndex().data(QFileSystemModel::FilePathRole).toString().toStdString() << std::endl;
  //std::cout << listView->currentIndex().data(QFileSystemModel::FileNameRole).toString().toStdString() << std::endl;

  this->listView->setRootIndex(model->index(listView->currentIndex().data(QFileSystemModel::FilePathRole).toString()));

  this->lblPath->setText(listView->currentIndex().data(QFileSystemModel::FilePathRole).toString());
}

void Form::on_listView_clicked(const QModelIndex & index)
{
  this->lblFile->setText(listView->currentIndex().data(QFileSystemModel::FilePathRole).toString());

  if(!this->model->isDir(this->listView->currentIndex()))
    {
    this->lblValid->setText("Valid");
    }
  else
    {
    this->lblValid->setText("Not Valid");
    }
}
