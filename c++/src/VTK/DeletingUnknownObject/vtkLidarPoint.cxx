#include "vtkLidarPoint.h"
#include "vtkRay.h"

#include "vtkSmartPointer.h"
#include "vtkObjectFactory.h" //for new() macro

vtkCxxRevisionMacro(vtkLidarPoint, "$Revision: 1.1 $");
vtkStandardNewMacro(vtkLidarPoint);

vtkCxxSetObjectMacro(vtkLidarPoint, Ray, vtkRay);

vtkLidarPoint::vtkLidarPoint()
{
  this->Ray = NULL;
}

vtkLidarPoint::~vtkLidarPoint() 
{
  if (this->Ray)
    {
    this->Ray->Delete();
    }
}


void vtkLidarPoint::PrintSelf(vtkstd::ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
