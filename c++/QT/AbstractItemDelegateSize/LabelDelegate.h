#ifndef LABELDELEGATE_H
#define LABELDELEGATE_H

#include <QStyledItemDelegate>

class LabelDelegate : public QStyledItemDelegate
{
   Q_OBJECT
   
signals:
  void UpdateSize(int, int, int, int) const;
  
public:
  LabelDelegate(){}
  
  void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;

};



#endif
