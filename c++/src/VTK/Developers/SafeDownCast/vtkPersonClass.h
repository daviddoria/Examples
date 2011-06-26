#ifndef __vtkPersonClass_h
#define __vtkPersonClass_h

#include "vtkDataObject.h"

#include <vtkstd/string>

class vtkPersonClass : public vtkDataObject
{
public:
  vtkTypeRevisionMacro(vtkPersonClass,vtkDataObject);
  static vtkPersonClass *New();
  void PrintSelf(ostream& os, vtkIndent indent);
  
  vtkstd::string GetName();
  void SetName(vtkstd::string );

  
  void CopyPerson(vtkPersonClass*);
  
protected:
  vtkPersonClass();
  ~vtkPersonClass();
  
  vtkstd::string Name;
  
private:
  vtkPersonClass(const vtkPersonClass&);  // Not implemented.
  void operator=(const vtkPersonClass&);  // Not implemented.

  
};

#endif
