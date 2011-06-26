#ifndef __vtkMyClass_h
#define __vtkMyClass_h

#include "vtkPolyDataAlgorithm.h" //superclass

template <typename T> class vtkDenseArray;

class vtkMyClass : public vtkPolyDataAlgorithm
{

public:
  static vtkMyClass *New();
  vtkTypeRevisionMacro(vtkMyClass,vtkObject);
  void PrintSelf(ostream &os, vtkIndent indent);
		
protected:
  vtkMyClass();
  ~vtkMyClass();
  int RequestData(vtkInformation *, vtkInformationVector **, vtkInformationVector *); //the function that makes this class work with the vtk pipeline
		
private:
  vtkDenseArray<double>* OutputGrid;
};

#endif