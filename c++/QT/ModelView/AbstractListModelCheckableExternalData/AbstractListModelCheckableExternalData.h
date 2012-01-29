#ifndef AbstractListModelCheckable_H
#define AbstractListModelCheckable_H

#include <QAbstractListModel>
#include <QVector>

#include "Item.h"

class AbstractListModelCheckable : public QAbstractListModel
{
public:
  AbstractListModelCheckable();

  QVariant data(const QModelIndex& index, int role) const;
  
  bool setData (const QModelIndex &index, const QVariant &value, int role);

  int rowCount(const QModelIndex  &parent=QModelIndex() ) const;

  int setItems(QVector<Item>* const items);

  Qt::ItemFlags flags (const QModelIndex &index ) const;

protected:
  QVector<Item>* Items;
};

#endif
