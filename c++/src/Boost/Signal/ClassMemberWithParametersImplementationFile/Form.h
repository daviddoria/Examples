#ifndef FORM_H
#define FORM_H

#include "InteractorStyle.h"

class Form
{
private:
  InteractorStyle MyInteractorStyle;
  float Data;
public:
  Form();

  void DataUpdated(float data);

  void TellInteractorToUpdate();

  void OutputData();

};

#endif