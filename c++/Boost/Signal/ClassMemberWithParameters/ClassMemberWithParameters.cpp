#include <boost/signal.hpp>
#include <boost/bind.hpp>

#include <iostream>

class InteractorStyle
{
public:
  boost::signal<void (float)> UpdateData;
};

class Form
{
private:
  InteractorStyle MyInteractorStyle;
  float Data;
public:
  Form()
  {
    MyInteractorStyle.UpdateData.connect(boost::bind(&Form::DataUpdated, this, _1));
  }

  void DataUpdated(float data)
  {
    this->Data = data;
    std::cout << "Data updated!" << std::endl;
  }

  void TellInteractorToUpdate()
  {
    // This is just for the demo - it would really be caused by a user event
    MyInteractorStyle.UpdateData(5.1);
  }

  void OutputData()
  {
    std::cout << "Data: " << Data << std::endl;
  }

};

int main()
{
  Form MyForm;
  MyForm.TellInteractorToUpdate(); // This is just for the demo - it would really be caused by a user event
  MyForm.OutputData();
}