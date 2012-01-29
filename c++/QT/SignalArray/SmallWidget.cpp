#include "SmallWidget.h"

#include <iostream>

SmallWidget::SmallWidget()
{
}

void SmallWidget::EmitSignal()
{
  emit mysignal();
}
