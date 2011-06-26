#ifndef __vtkLawyerClass_h
#define __vtkLawyerClass_h

#include "vtkPersonClass.h"

#include <vtkstd/string>

class vtkLawyerClass : public vtkPersonClass
{
public:
  vtkTypeRevisionMacro(vtkLawyerClass,vtkPersonClass);
  static vtkLawyerClass *New();
  void PrintSelf(ostream& os, vtkIndent indent);
  
  void CopyLawyer(vtkLawyerClass*);
  void CopyPerson(vtkPersonClass*);
  
  vtkstd::string GetLawSchool();
  void SetLawSchool(vtkstd::string );
  
protected:
  vtkLawyerClass();
  ~vtkLawyerClass();
  
private:
  vtkLawyerClass(const vtkLawyerClass&);  // Not implemented.
  void operator=(const vtkLawyerClass&);  // Not implemented.

  
  vtkstd::string LawSchool;
  
};

#endif
