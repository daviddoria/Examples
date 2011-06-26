// .NAME vtkTestFilter
// .SECTION Description
// vtkTestFilter

#ifndef __vtkTestFilter_h
#define __vtkTestFilter_h

#include "vtkPassInputTypeAlgorithm.h"

class vtkTestFilter : public vtkPassInputTypeAlgorithm 
{
public:
  vtkTypeMacro(vtkTestFilter,vtkPassInputTypeAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

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
