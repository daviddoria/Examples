#ifndef __vtkPersonClass_h
#define __vtkPersonClass_h

#include "vtkDataObject.h"

class vtkPersonClass : public vtkDataObject
{
public:
  vtkTypeRevisionMacro(vtkPersonClass,vtkDataObject);
  static vtkPersonClass *New();
  void PrintSelf(ostream& os, vtkIndent indent);
  void ShallowCopy(vtkPersonClass*);
  
  vtkSetMacro(Value, double);
  vtkGetMacro(Value, double);
  
protected:
  vtkPersonClass();
  ~vtkPersonClass();
  
private:
  vtkPersonClass(const vtkPersonClass&);  // Not implemented.
  void operator=(const vtkPersonClass&);  // Not implemented.

  double Value;
};

#endif
