#include "AbstractListModelFileDialog.h"

#include <iostream>

int AbstractListModel::setItems(const QVector<Item>& items)
{
  emit beginResetModel();
  this->Items = items;
  emit endResetModel();
}

Qt::ItemFlags AbstractListModel::flags (const QModelIndex  &index ) const
{
  if (index.row() < 0 || index.row() >= rowCount() || !index.isValid())
    {
    return Qt::NoItemFlags;
    }
  return Qt::ItemIsEditable | Qt::ItemIsEnabled;
}

AbstractListModel::AbstractListModel() : QAbstractListModel()
{

}

int AbstractListModel::rowCount(const QModelIndex& parent) const
{
  return this->Items.size();
}

QVariant AbstractListModel::data (const QModelIndex  &index , int role ) const
{
  if (index.row() < 0 || index.row() >= rowCount() || !index.isValid())
    {
    return QVariant();
    }

  if (role == Qt::DisplayRole || role == Qt::EditRole)
    {
    return this->Items.at(index.row()).Name;
    }

  return QVariant();
}

bool AbstractListModel::setData (const QModelIndex &index, const QVariant &value, int role)
{
  if (index.row() < 0 || index.row() >= rowCount() || !index.isValid())
    {
    return false;
    }

  if (role == Qt::DisplayRole || role == Qt::EditRole)
    {
    this->Items[index.row()].Name = value.toString();
    }
  emit dataChanged(index, index);
  return true;
}
