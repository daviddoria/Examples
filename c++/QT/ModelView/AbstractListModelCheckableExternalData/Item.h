#ifndef ITEM_H
#define ITEM_H

class Item
{
public:
  Item(const QString& name = QString(), const Qt::CheckState displayed = Qt::Unchecked) :
  Name(name), Displayed(displayed){}
  QString Name;
  Qt::CheckState Displayed;
};

#endif
