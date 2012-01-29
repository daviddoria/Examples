#ifndef MyTableModel_H
#define MyTableModel_H

#include <QAbstractTableModel>

#include <vector>

class MyTableModel : public QAbstractTableModel
{
public:
  MyTableModel();
  
  int rowCount(const QModelIndex& parent) const;
  int columnCount(const QModelIndex& parent) const;
  QVariant data(const QModelIndex& index, int role) const;
  Qt::ItemFlags flags ( const QModelIndex & index ) const;
};

#endif
