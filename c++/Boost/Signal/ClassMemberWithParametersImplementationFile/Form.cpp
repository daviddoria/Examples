#include "Form.h"

#include <iostream>

#include <boost/bind.hpp>

Form::Form()
{
  MyInteractorStyle.UpdateData.connect(boost::bind(&Form::DataUpdated, this, _1));
}

void Form::DataUpdated(float data)
{
  this->Data = data;
  std::cout << "Data updated!" << std::endl;
}

void Form::TellInteractorToUpdate()
{
  // This is just for the demo - it would really be caused by a user event
  MyInteractorStyle.UpdateData(5.1);
}

void Form::OutputData()
{
  std::cout << "Data: " << Data << std::endl;
}