#ifndef FloatSlider_H
#define FloatSlider_H

#include <QSlider>
#include <QtDesigner/QDesignerExportWidget>

#include "ui_FloatSlider.h"

class QDESIGNER_WIDGET_EXPORT FloatSlider : public QSlider, public Ui::FloatSlider
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
