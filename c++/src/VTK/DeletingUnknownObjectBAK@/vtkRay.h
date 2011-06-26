#ifndef __vtkRay_h
#define __vtkRay_h

class vtkTransform;
#include "vtkObject.h" //superclass

class vtkRay : public vtkObject
{
public:
  static vtkRay *New();
  vtkTypeRevisionMacro(vtkRay,vtkObject);
    
  void PrintSelf(vtkstd::ostream &os, vtkIndent indent);
		
protected:
  vtkRay();
  ~vtkRay();
private:
  vtkRay(const vtkRay&);  // Not implemented.
  void operator=(const vtkRay&);  // Not implemented.
  
};

#endif
