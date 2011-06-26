#include "InteractorStyle.h"
#include "vtkObjectFactory.h"

#include <vtkBoxWidget2.h>

vtkStandardNewMacro(InteractorStyle);

InteractorStyle::InteractorStyle()
{
  this->BoxWidget = vtkSmartPointer<vtkBoxWidget2>::New();
}

void InteractorStyle::Initialize()
{
  this->BoxWidget->SetInteractor(this->Interactor);
  this->BoxWidget->On();
}