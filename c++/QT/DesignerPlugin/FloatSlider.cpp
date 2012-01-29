#include <iostream>

#include "FloatSlider.h"

FloatSlider::FloatSlider(QWidget *parent) : QSlider(parent)
{
  setupUi(this);
}

float FloatSlider::GetValue()
{
  return this->MinValue + (this->MaxValue - this->MinValue) * static_cast<float>(this->value()) / 100.0f;
}
