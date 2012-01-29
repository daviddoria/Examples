#ifndef ClickableLabel_H
#define ClickableLabel_H

#include <QLabel>

class ClickableLabel : public QLabel
{
  Q_OBJECT
public:
  
  void mousePressEvent(QMouseEvent* pMouseEvent);
  unsigned int Id;

signals:
  void ClickedSignal(const unsigned int);
};

#endif
