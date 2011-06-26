#include "vtkRay.h"

#include "vtkObjectFactory.h" //for new() macro
#include "vtkMath.h"
#include "vtkTransform.h"


vtkCxxRevisionMacro(vtkRay, "$Revision: 1.1 $");
vtkStandardNewMacro(vtkRay);

vtkRay::vtkRay()
{
	
}

vtkRay::~vtkRay() 
{
}

void vtkRay::PrintSelf(vtkstd::ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
