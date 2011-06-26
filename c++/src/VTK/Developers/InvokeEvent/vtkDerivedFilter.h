#ifndef __vtkDerivedFilter_h
#define __vtkDerivedFilter_h

#include "vtkTestFilter.h"

class vtkDerivedFilter : public vtkTestFilter
{
public:
  vtkTypeMacro(vtkDerivedFilter,vtkTestFilter);

  static vtkDerivedFilter *New();

protected:
  vtkDerivedFilter(){}
  ~vtkDerivedFilter(){}

  void Test();
private:
  vtkDerivedFilter(const vtkDerivedFilter&);  // Not implemented.
  void operator=(const vtkDerivedFilter&);  // Not implemented.

};

#endif
