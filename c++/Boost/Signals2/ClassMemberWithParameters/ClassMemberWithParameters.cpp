#include <boost/signals2/signal.hpp>
#include <boost/bind.hpp>

#include <iostream>

class InteractorStyle
{
public:
  boost::signals2::signal<void (float, int)> UpdateData;
};

class Form
{
private:
  InteractorStyle MyInteractorStyle;
  float FloatData;
  int IntData;

public:
  Form()
  {
    MyInteractorStyle.UpdateData.connect(boost::bind(&Form::DataUpdated, this, _1, _2));
  }

  void DataUpdated(float floatData, int intData)
  {
    this->FloatData = floatData;
    this->IntData = intData;
    std::cout << "Data updated!" << std::endl;
  }

  void TellInteractorToUpdate()
  {
    // This is just for the demo - it would really be caused by a user event
    MyInteractorStyle.UpdateData(5.1, 3);
  }

  void OutputData()
  {
    std::cout << "Float data: " << this->FloatData << std::endl;
    std::cout << "Int data: " << this->IntData << std::endl;
  }

};

int main()
{
  Form MyForm;
  MyForm.TellInteractorToUpdate(); // This is just for the demo - it would really be caused by a user event
  MyForm.OutputData();
}