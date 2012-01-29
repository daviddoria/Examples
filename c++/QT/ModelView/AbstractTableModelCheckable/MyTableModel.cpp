#include "MyTableModel.h"

#include <sstream>

MyTableModel::MyTableModel() : QAbstractTableModel()
{
  std::vector<Item> column1;
  Item item(10, Qt::Checked);
  column1.push_back(item);
  item.value = 20; item.state = Qt::Checked;
  column1.push_back(item);
  item.value = 30; item.state = Qt::Checked;
  column1.push_back(item);
  item.value = 40; item.state = Qt::Checked;
  column1.push_back(item);
  
  Columns.push_back(column1);
  
  std::vector<Item> column2;
  item.value = 50; item.state = Qt::Unchecked;
  column2.push_back(item);
  item.value = 60; item.state = Qt::Unchecked;
  column2.push_back(item);
  item.value = 70; item.state = Qt::Unchecked;
  column2.push_back(item);
  item.value = 80; item.state = Qt::Unchecked;
  column2.push_back(item);
  
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
    return Columns[index.column()][index.row()].value;
    }
  if (index.column() == 1 && role == Qt::CheckStateRole)
    {
    return this->Columns[index.column()][index.row()].state;
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


bool MyTableModel::setData(const QModelIndex & index, const QVariant & value, int role)
{
  if (index.column() == 1 && role == Qt::CheckStateRole)
    {
    this->Columns[index.column()][index.row()].state = static_cast<Qt::CheckState>(value.toUInt());
    }

  emit dataChanged(index, index);
  return true;
}

Qt::ItemFlags MyTableModel::flags(const QModelIndex& index) const
{
  Qt::ItemFlags returnFlags = QAbstractTableModel::flags(index);

  if (index.column() == 1)
    {
    returnFlags |= Qt::ItemIsUserCheckable;
    }

  return returnFlags;
}
