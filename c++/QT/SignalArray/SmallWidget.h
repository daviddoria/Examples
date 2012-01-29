#ifndef SmallWidget_H
#define SmallWidget_H

#include <QObject>

class SmallWidget : public QObject
{
Q_OBJECT

public:
  SmallWidget();

  void EmitSignal();

signals:
  void mysignal();
  void mysignalVector(QVector<double>);

};

#endif