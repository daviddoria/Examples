#include "vtkPersonClass.h"

#include "vtkObjectFactory.h"

vtkCxxRevisionMacro(vtkPersonClass, "$Revision: 1.70 $");
vtkStandardNewMacro(vtkPersonClass);

vtkPersonClass::vtkPersonClass()
{
  
}

vtkPersonClass::~vtkPersonClass()
{

}

void vtkPersonClass::PrintSelf(ostream& os, vtkIndent indent)
{
  //vtkstd::cout << "ImageCamera Superclass: " << this->Superclass->GetClassName() << vtkstd::endl;
  //vtkstd::cout << "ImageCamera Superclass: " << ::GetClassName() << vtkstd::endl;
  this->Superclass::PrintSelf(os,indent);

  os << indent << "PersonValue: " << this->Value << "\n";
  
}


void vtkPersonClass::ShallowCopy(vtkPersonClass* P)
{
  this->Value = P->GetValue();
}
