#include "MyTableModel.h"

#include <QSize>

#include <iostream>
#include <sstream>

MyTableModel::MyTableModel() : QAbstractTableModel()
{
  std::vector<float> column1;
  column1.push_back(10);
  column1.push_back(20);
  column1.push_back(30);
  column1.push_back(40);
  
  Columns.push_back(column1);
  
  std::vector<float> column2;
  column2.push_back(50);
  column2.push_back(60);
  column2.push_back(70);
  column2.push_back(80);
  
  Columns.push_back(column2);
}

int MyTableModel::rowCount(const QModelIndex& parent) const
{
  return Columns[0].size();
}

int MyTableModel::columnCount(const QModelIndex& parent) const
{
  return Columns.size();
}

QVariant MyTableModel::data(const QModelIndex& index, int role) const
{
  if(role == Qt::DisplayRole)
    {
    return Columns[index.column()][index.row()];
    }
  else if(role == Qt::SizeHintRole && index.column() == 1)
    {
    std::cout << "Setting size..." << std::endl;
    return QSize(300,300); // This does not work.
    }
  return QVariant::Invalid;
}

QVariant MyTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
  
  if(role == Qt::DisplayRole)
    {
    std::stringstream ss;
    if(orientation == Qt::Horizontal)
      {
      ss << "H_" << section;
      return QString(ss.str().c_str());
      }
    else if(orientation == Qt::Vertical)
      {
      ss << "V_" << section;
      return QString(ss.str().c_str());
      }
    
    }
    
  return QVariant::Invalid;
}
