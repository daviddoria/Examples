#include <iostream>
#include <sstream>

template <typename T>
MyInsideForm<T>::MyInsideForm(QWidget *parent)
{

}

template <typename T>
void MyInsideForm<T>::btnButton_clicked()
{
  this->SetValue(2.1);
  std::stringstream ss;
  ss << this->value;
  this->lblLabel->setText(ss.str().c_str());
}
