#ifndef __vtkTestSource_h
#define __vtkTestSource_h

#include "vtkAlgorithm.h"

#include "vtkTest.h"

class vtkTestSource : public vtkAlgorithm 
{
public:
  vtkTypeRevisionMacro(vtkTestSource,vtkAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkTestSource *New();
	
  vtkTest* GetOutput();
  
protected:
  vtkTestSource();
  ~vtkTestSource();
  
  int FillOutputPortInformation( int port, vtkInformation* info );
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *);

private:
  vtkTestSource(const vtkTestSource&);  // Not implemented.
  void operator=(const vtkTestSource&);  // Not implemented.

};

#endif
