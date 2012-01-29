#include "MyTableModel.h"

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
  
}

Qt::ItemFlags MyTableModel::flags(const QModelIndex& index) const
{
  return Qt::ItemIsEditable | Qt::ItemIsSelectable | Qt::ItemIsEnabled;
}

void MyTableModel::ChangeData()
{
  for(unsigned int i = 0; i < this->Columns[0].size(); ++i)
    {
    this->Columns[0][i]++;
    this->Columns[1][i]+=2;
    }

  beginResetModel();
  endResetModel();
}
