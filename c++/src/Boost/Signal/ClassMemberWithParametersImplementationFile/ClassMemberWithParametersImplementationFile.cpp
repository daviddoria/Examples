#include <boost/signal.hpp>
#include <boost/bind.hpp>

#include <iostream>

#include "InteractorStyle.h"
#include "Form.h"


int main()
{
  Form MyForm;
  MyForm.TellInteractorToUpdate(); // This is just for the demo - it would really be caused by a user event
  MyForm.OutputData();
}