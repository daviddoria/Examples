#ifndef AbstractListModelCheckable_H
#define AbstractListModelCheckable_H

#include <QAbstractListModel>

#include <QVector>

class Item
{
public:
  Item(const QString& name = QString(), const Qt::CheckState displayed = Qt::Unchecked) :
  Name(name), Displayed(displayed){}
  QString Name;
  Qt::CheckState Displayed;
};

class AbstractListModelCheckable : public QAbstractListModel
{
public:
  AbstractListModelCheckable();

  QVariant data(const QModelIndex& index, int role) const;
  
  bool setData (const QModelIndex &index, const QVariant &value, int role);

  int rowCount(const QModelIndex  &parent=QModelIndex() ) const;

  int setItems(const QVector<Item>& items);

  Qt::ItemFlags flags (const QModelIndex  &index ) const;

protected:
  QVector<Item> Items;
};

#endif
