#ifndef AbstractListModel_H
#define AbstractListModel_H

#include <QAbstractListModel>

#include <QVector>

class Item
{
public:
  Item(const QString& name = QString()) :  Name(name){}
  QString Name;
};

class AbstractListModel : public QAbstractListModel
{
public:
  AbstractListModel();

  QVariant data(const QModelIndex& index, int role) const;
  
  bool setData (const QModelIndex &index, const QVariant &value, int role);

  int rowCount(const QModelIndex  &parent=QModelIndex() ) const;

  int setItems(const QVector<Item>& items);

  Qt::ItemFlags flags (const QModelIndex  &index ) const;

protected:
  QVector<Item> Items;
};

#endif
