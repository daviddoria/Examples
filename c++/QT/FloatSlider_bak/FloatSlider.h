#ifndef FloatSlider_H
#define FloatSlider_H

#include "ui_FloatSlider.h"

class FloatSlider : public QSlider, public Ui::FloatSlider
{
Q_OBJECT

signals:
  void valueChanged(float);

public:
  FloatSlider(QWidget *parent = 0);

  float GetValue();

protected:
  float MinValue;
  float MaxValue;
};

#endif
