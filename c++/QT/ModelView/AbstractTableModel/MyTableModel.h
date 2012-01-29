#ifndef MYTABLEMODEL_H
#define MYTABLEMODEL_H

#include <QAbstractTableModel>

#include <vector>

class MyTableModel : public QAbstractTableModel
{
public:
  MyTableModel();
  
  int rowCount(const QModelIndex& parent) const;
  int columnCount(const QModelIndex& parent) const;
  QVariant data(const QModelIndex& index, int role) const;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const;
  
protected:
  std::vector<std::vector<float> > Columns;
};

#endif
