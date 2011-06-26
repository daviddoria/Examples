#ifndef __vtkLawyerClass_h
#define __vtkLawyerClass_h

#include "vtkPersonClass.h"

class vtkLawyerClass : public vtkPersonClass
{
public:
  vtkTypeRevisionMacro(vtkLawyerClass,vtkPersonClass);
  static vtkLawyerClass *New();
  void PrintSelf(ostream& os, vtkIndent indent);
  //void ShallowCopy(vtkLawyerClass*);
  
  vtkSetMacro(Value2, double);
  vtkGetMacro(Value2, double);
  
  
protected:
  vtkLawyerClass();
  ~vtkLawyerClass();
  
private:
  vtkLawyerClass(const vtkLawyerClass&);  // Not implemented.
  void operator=(const vtkLawyerClass&);  // Not implemented.

  double Value2;
};

#endif
