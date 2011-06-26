#ifndef __vtkMyClass_h
#define __vtkMyClass_h

#include "vtkPolyDataAlgorithm.h" //superclass

template <typename T> class vtkDenseArray;
template <typename T> class vtkSmartPointer;

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
  vtkSmartPointer<vtkDenseArray<double> > OutputGrid; //outer vector is size NumberOfThetaPoints, inner vector is size NumberOfPhiPoints
};

#endif

