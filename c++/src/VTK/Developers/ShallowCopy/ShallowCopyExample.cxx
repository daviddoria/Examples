#include <vtkSmartPointer.h>

#include "vtkPersonClass.h"
#include "vtkLawyerClass.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkPersonClass> Person = vtkSmartPointer<vtkPersonClass>::New();
  Person->SetValue(5.6);
  vtkstd::cout << "Person: " << *Person << vtkstd::endl;
  //vtkstd::cout << "Person value: " << Person->GetValue() << vtkstd::endl;
  
  //vtkLawyerClass* Lawyer = vtkLawyerClass::SafeDownCast(Person);
  vtkSmartPointer<vtkLawyerClass> Lawyer = vtkSmartPointer<vtkLawyerClass>::New();
  Lawyer->ShallowCopy(Person);
  Lawyer->SetValue2(7.1);
  //vtkstd::cout << "Lawyer value: " << Lawyer->GetValue() << " " << Lawyer->GetValue2() << vtkstd::endl;
  vtkstd::cout << "Lawyer: " << *Lawyer << vtkstd::endl;
  
  return 0;
}
