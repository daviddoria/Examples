#include "AbstractListModelCheckableExternalDataSafe.h"

#include <iostream>

int AbstractListModelCheckable::setItems(QVector<Item>* const items)
{
  emit beginResetModel();
  this->Items = items;
  emit endResetModel();
}

Qt::ItemFlags AbstractListModelCheckable::flags (const QModelIndex  &index ) const
{
  if (index.row() < 0 || index.row() >= rowCount() || !index.isValid())
    {
    return Qt::NoItemFlags;
    }
  //return Qt::ItemIsUserCheckable | Qt::ItemIsEditable | Qt::ItemIsEnabled; // ItemIsEditable will let you change the text. You can still change the check boxes even without ItemIsEditable.
  return Qt::ItemIsUserCheckable | Qt::ItemIsEnabled;
}

AbstractListModelCheckable::AbstractListModelCheckable() : QAbstractListModel()
{

}

int AbstractListModelCheckable::rowCount(const QModelIndex& parent) const
{
  return this->Items->size();
}

QVariant AbstractListModelCheckable::data (const QModelIndex  &index , int role ) const
{
  if (index.row() < 0 || index.row() >= rowCount() || !index.isValid())
    {
    return QVariant();
    }

  if (role == Qt::DisplayRole || role == Qt::EditRole)
    {
    return this->Items->at(index.row()).Name;
    }

  if(role == Qt::CheckStateRole)
    {
    return this->Items->at(index.row()).Displayed;
    }

  return QVariant();
}

bool AbstractListModelCheckable::setData (const QModelIndex &index, const QVariant &value, int role)
{
  if (index.row() < 0 || index.row() >= rowCount() || !index.isValid())
    {
    return false;
    }

  if(role == Qt::CheckStateRole)
    {
    (*this->Items)[index.row()].Displayed = static_cast<Qt::CheckState>(value.toUInt());
    }

  emit dataChanged(index, index);
  return true;
}
