#include <QDir>
#include <QFileSystemModel>
#include <QWidget>

#include <iostream>

#include "FileSelectionWidget.h"

FileSelectionWidget::FileSelectionWidget(QWidget *parent)
    : QWidget(parent)
{
  std::cout << "Current path: " << QDir::currentPath().toStdString() << std::endl;
  setupUi(this); // Otherwise 'listView' seems to be undefined
    
  this->model = new QFileSystemModel;
  this->model->setRootPath(QDir::currentPath());
  this->listView->setModel(model);
  this->listView->setRootIndex(model->index(QDir::currentPath()));

  this->lblPath->setText(QDir::currentPath());
  this->lblFile->setText(listView->currentIndex().data(QFileSystemModel::FilePathRole).toString());
  //this->listView->setCurrentIndex(model->index(QDir::currentPath()));
  
}

void FileSelectionWidget::on_listView_doubleClicked(const QModelIndex & index)
{
  std::cout << listView->currentIndex().data(QFileSystemModel::FilePathRole).toString().toStdString() << std::endl;
  //std::cout << listView->currentIndex().data(QFileSystemModel::FileNameRole).toString().toStdString() << std::endl;

  if(this->model->isDir(this->listView->currentIndex())) // A directory was selected
    {
    this->listView->setRootIndex(model->index(listView->currentIndex().data(QFileSystemModel::FilePathRole).toString()));

    this->lblPath->setText(listView->currentIndex().data(QFileSystemModel::FilePathRole).toString());
    }
  else // A file was selected
    {
    on_listView_clicked(index);
    }
}

void FileSelectionWidget::on_listView_clicked(const QModelIndex & index)
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
