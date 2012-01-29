#ifndef LISTMODEL_H
#define LISTMODEL_H

#include <QAbstractListModel>

#include <vector>

class Item
{
public:
  Item(const std::string& name, const bool displayed, const bool saved) :
  Name(name), Displayed(displayed), Saved(saved){}
  std::string Name;
  bool Displayed;
  bool Saved;
};

class ListModel : public QAbstractListModel
{
public:
  ListModel();

  int rowCount(const QModelIndex& parent) const;
  
  QVariant data(const QModelIndex& index, int role) const;
  //QVariant headerData(int section, Qt::Orientation orientation, int role) const;

protected:
  std::vector<Item> Items;
};

#endif
