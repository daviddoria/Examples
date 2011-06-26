#ifndef __vtkTest_h
#define __vtkTest_h

#include "vtkObject.h"

class vtkInformation;
class vtkInformationVector;

class vtkTest : public vtkObject
{
public:
  vtkTypeRevisionMacro(vtkTest,vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkTest *New();
  	
  vtkSetMacro(Value,double);
  vtkGetMacro(Value,double);
protected:
  vtkTest();
  ~vtkTest();
  
private:
  vtkTest(const vtkTest&);  // Not implemented.
  void operator=(const vtkTest&);  // Not implemented.
  
  double Value;
};

#endif
