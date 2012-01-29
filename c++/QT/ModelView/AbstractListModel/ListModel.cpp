#include "ListModel.h"

#include <sstream>

ListModel::ListModel() : QAbstractListModel()
{
  this->Items.push_back(Item("Test0", true, true));
  this->Items.push_back(Item("Test1", false, false));
  this->Items.push_back(Item("Test2", true, false));
}

int ListModel::rowCount(const QModelIndex& parent) const
{
  return this->Items.size();
}

QVariant ListModel::data(const QModelIndex& index, int role) const
{
  if(role == Qt::DisplayRole)
    {
    return Items[index.row()].Displayed;
    }
  return QVariant::Invalid;
}
/*
QVariant ListModel::headerData(int section, Qt::Orientation orientation, int role) const
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
}*/
