#ifndef __vtkTestFilter_h
#define __vtkTestFilter_h
 
#include "vtkPolyDataAlgorithm.h"
 
class vtkTestFilter : public vtkPolyDataAlgorithm 
{
public:
  vtkTypeMacro(vtkTestFilter,vtkAlgorithm);
 
protected:
  vtkTestFilter(){}
  ~vtkTestFilter(){}
 
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

  virtual void Test() = 0;
private:
  vtkTestFilter(const vtkTestFilter&);  // Not implemented.
  void operator=(const vtkTestFilter&);  // Not implemented.
 
};
 
#endif
