#include <QtGui>

#include <iostream>

#include "form.h"

#include <QItemSelectionModel>
#include <QStringListModel>

Form::Form(QWidget *parent) : QMainWindow(parent)
{
  setupUi(this);
  std::cout << "Current path: " << QDir::currentPath().toStdString() << std::endl;

  //this->StringList = new QStringList("a" << "b" << "c");
  //this->StringList = new QStringList("a", "b", "c");
  this->StringList = new QStringList;
  *(this->StringList) << "a";
  *(this->StringList) << "b";
  *(this->StringList) << "c";
    
  this->Model = new QStringListModel;
  this->Model->setStringList(*this->StringList);
  this->listView->setModel(this->Model);

  this->SelectionModel = new QItemSelectionModel(this->Model);
  this->listView->setSelectionModel(this->SelectionModel);

  connect(this->SelectionModel, SIGNAL(selectionChanged (const QItemSelection&, const QItemSelection&)),
          this, SLOT(slot_selectionChanged(QItemSelection,QItemSelection)));
}

void Form::slot_selectionChanged (const QItemSelection  &selected, const QItemSelection  &deselected )
{
  for(unsigned int i = 0; i < selected.indexes().size(); ++i)
    {
    std::cout << selected.at(i).topLeft().row() << " ";
    }
  std::cout << std::endl;
}
