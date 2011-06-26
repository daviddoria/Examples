#include <vtkSmartPointer.h>

#include "vtkPersonClass.h"
#include "vtkLawyerClass.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkPersonClass> David = vtkSmartPointer<vtkPersonClass>::New();
  David->SetName("David");
  vtkstd::cout << "David: " << *David << vtkstd::endl;
  
  vtkSmartPointer<vtkLawyerClass> Lawyer = vtkSmartPointer<vtkLawyerClass>::New();
  Lawyer->CopyPerson(David);
  Lawyer->SetLawSchool("Yale");
  vtkstd::cout << "Lawyer: " << *Lawyer << vtkstd::endl;
  
  return 0;
}
