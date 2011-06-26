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

  os << indent << "vtkLawyerClass::PrintSelf \n";
  os << indent << "LawSchool: " << this->LawSchool << "\n";
  
}

void vtkLawyerClass::SetLawSchool(vtkstd::string l)
{
  this->LawSchool = l;
}

vtkstd::string vtkLawyerClass::GetLawSchool()
{
  return this->LawSchool;
}

void vtkLawyerClass::CopyPerson(vtkPersonClass* p)
{
  //this->Name = p->Name;
  this->Name = p->GetName();
}

void vtkLawyerClass::CopyLawyer(vtkLawyerClass* l)
{
  this->CopyPerson(dynamic_cast<vtkPersonClass*>(l));
  this->LawSchool = l->GetLawSchool();
}