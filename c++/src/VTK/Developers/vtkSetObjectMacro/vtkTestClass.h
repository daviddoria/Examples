#ifndef __vtkTestClass_h
#define __vtkTestClass_h

#include "vtkDataObject.h"

class vtkMatrix3x3;

class vtkTestClass : public vtkDataObject
{
public:
  vtkTypeRevisionMacro(vtkTestClass,vtkDataObject);
  static vtkTestClass *New();
	
  /*
  vtkSetObjectMacro(M, vtkMatrix3x3*);
  vtkGetObjectMacro(M, vtkMatrix3x3*);
  */
  
  vtkSetMacro(M, vtkMatrix3x3*);
  vtkGetMacro(M, vtkMatrix3x3*);
  
  
protected:
  vtkTestClass();
  ~vtkTestClass();
  
private:
  vtkTestClass(const vtkTestClass&);  // Not implemented.
  void operator=(const vtkTestClass&);  // Not implemented.

  vtkMatrix3x3* M;
};

#endif
