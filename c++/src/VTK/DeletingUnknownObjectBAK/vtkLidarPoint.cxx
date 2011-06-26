#include "vtkLidarPoint.h"
#include "vtkRay.h"

#include "vtkSmartPointer.h"
#include "vtkObjectFactory.h" //for new() macro

vtkStandardNewMacro(vtkLidarPoint);

vtkLidarPoint::vtkLidarPoint()
{
  this->Ray = NULL;
}

vtkLidarPoint::~vtkLidarPoint() 
{
  this->Ray->Delete();
}


void vtkLidarPoint::PrintSelf(vtkstd::ostream &os, vtkIndent indent)
{

}

