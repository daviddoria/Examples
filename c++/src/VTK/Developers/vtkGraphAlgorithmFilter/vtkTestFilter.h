#ifndef __vtkTestFilter_h
#define __vtkTestFilter_h

#include "vtkGraphAlgorithm.h"

class vtkTestFilter : public vtkGraphAlgorithm 
{
public:
  vtkTypeMacro(vtkTestFilter,vtkGraphAlgorithm);
  
  static vtkTestFilter *New();
	
protected:
  vtkTestFilter(){}
  ~vtkTestFilter(){}
  
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  vtkTestFilter(const vtkTestFilter&);  // Not implemented.
  void operator=(const vtkTestFilter&);  // Not implemented.

};

#endif
