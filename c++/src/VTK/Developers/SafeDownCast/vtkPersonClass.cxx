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
  this->Superclass::PrintSelf(os,indent);

  os << indent << "vtkPersonClass::PrintSelf \n";
  os << indent << "Name: " << this->Name << "\n";
  
}

void vtkPersonClass::CopyPerson(vtkPersonClass* p)
{
  this->Name = p->Name;
}

vtkstd::string vtkPersonClass::GetName()
{
  return this->Name;
}


void vtkPersonClass::SetName(vtkstd::string n)
{
  this->Name = n;
}