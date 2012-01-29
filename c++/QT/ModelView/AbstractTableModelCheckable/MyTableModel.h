#ifndef MyTableModel_H
#define MyTableModel_H

#include <QAbstractTableModel>

#include <vector>

class Item
{
public:
  Item(const float v, const Qt::CheckState s) {value = v; state = s;}
  float value;
  Qt::CheckState state;
};

class MyTableModel : public QAbstractTableModel
{
public:
  MyTableModel();

  int rowCount(const QModelIndex& parent) const;
  int columnCount(const QModelIndex& parent) const;
  QVariant data(const QModelIndex& index, int role) const;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const;
  Qt::ItemFlags flags(const QModelIndex& index) const;
  bool setData(const QModelIndex & index, const QVariant & value, int role);

protected:
  std::vector<std::vector<Item> > Columns;
};

#endif
