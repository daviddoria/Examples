#ifndef MyTableWidgetItem_H
#define MyTableWidgetItem_H

#include <QTableWidgetItem>
#include <QObject>

class MyTableWidgetItem : public QObject, public QTableWidgetItem // These must be inherited in this order to prevent "staticMetaObject is not a member" errors!
{
  Q_OBJECT
public:
  
  void mousePressEvent(QGraphicsSceneMouseEvent* pMouseEvent);
  unsigned int Id;

signals:
  void ClickedSignal(const unsigned int);
};

#endif
