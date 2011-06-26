#include "vtkLawyerClass.h"

#include "vtkObjectFactory.h"

vtkCxxRevisionMacro(vtkLawyerClass, "$Revision: 1.70 $");
vtkStandardNewMacro(vtkLawyerClass);

vtkLawyerClass::vtkLawyerClass()
{
  
}

vtkLawyerClass::~vtkLawyerClass()
{

}

void vtkLawyerClass::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);

  os << indent << "LawyerValue: " << this->GetValue() << "\n";
  os << indent << "LawyerValue2: " << this->GetValue2() << "\n";
  
}
/*
void vtkLawyerClass::ShallowCopy(vtkLawyerClass* L)
{
  this->Value2 = L->GetValue2();
}
*/