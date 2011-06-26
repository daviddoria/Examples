#ifndef __vtkTest_h
#define __vtkTest_h

//#include "vtkObject.h"
#include "vtkDataObject.h"

class vtkInformation;
class vtkInformationVector;

//class vtkTest : public vtkObject
class vtkTest : public vtkDataObject
{
public:
  vtkTypeRevisionMacro(vtkTest,vtkObject);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkTest *New();
  	
  vtkSetMacro(Value,double);
  vtkGetMacro(Value,double);
  
  void ShallowCopy(vtkTest* t);
  
protected:
  vtkTest();
  ~vtkTest();
  
private:
  vtkTest(const vtkTest&);  // Not implemented.
  void operator=(const vtkTest&);  // Not implemented.
  
  double Value;
};

#endif
