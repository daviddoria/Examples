#ifndef LABELDELEGATE_H
#define LABELDELEGATE_H

#include <QStyledItemDelegate>

class LabelDelegate : public QStyledItemDelegate
{
   Q_OBJECT
public:
  LabelDelegate(){}
  
  void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;

  QSize sizeHint(const QStyleOptionViewItem & option, const QModelIndex & index) const;
};



#endif
