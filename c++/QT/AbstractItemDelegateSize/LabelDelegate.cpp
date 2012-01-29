#include "LabelDelegate.h"

#include <QPainter>

#include <iostream>

void LabelDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
  QPixmap pixmap = index.data(Qt::DisplayRole).value<QPixmap>();
  //std::cout << pixmap.width() << " " << pixmap.height() << std::endl;
  //std::cout << "rect: " << option.rect.width() << " " << option.rect.height() << std::endl;

  QRect rect = option.rect;
  //rect.adjust(rect.width()/3, 0, -rect.width()/3, 0);
  painter->drawPixmap(rect, pixmap, pixmap.rect());
  
  emit UpdateSize(index.row(), index.column(), pixmap.width(), pixmap.height());
}
