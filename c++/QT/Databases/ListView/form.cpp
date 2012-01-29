#include "form.h"

#include <iostream>

#include <QSqlDatabase>
#include <QStringList>
#include <QSqlQuery>
#include <QDebug>
#include <QSqlError>
#include <QVariant>
#include <QSqlTableModel>
#include <QSqlRecord>

MyForm::MyForm(QWidget *parent) : QWidget(parent)
{
  setupUi(this);
}

MyForm::~MyForm()
{
  if(this->TableModel)
    {
    delete this->TableModel;
    }
}

void MyForm::showEvent ( QShowEvent * event )
{
  QSqlQuery selectQuery;
  selectQuery.prepare("SELECT AssociateName FROM AssociateTable");

  bool selectSuccess = selectQuery.exec();
  if(!selectSuccess)
    {
    std::cerr << "Error selecting associates!" << std::endl;
    std::cerr << "Last error: " << selectQuery.lastError().text().toStdString() << std::endl;
    return;
    }

  this->TableModel = new QSqlTableModel;
  this->TableModel->setTable("AssociateTable");
  this->TableModel->select();

  this->listView->setModel(this->TableModel);

  this->listView->setModelColumn(1);
}
