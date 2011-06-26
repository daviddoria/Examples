#include "vtkRay.h"

#include "vtkObjectFactory.h" //for new() macro
#include "vtkMath.h"
#include "vtkTransform.h"
#include "vtkSmartPointer.h"

vtkCxxRevisionMacro(vtkRay, "$Revision: 1.1 $");
vtkStandardNewMacro(vtkRay);

vtkRay::vtkRay()
{
	this->Transform = vtkSmartPointer<vtkTransform>::New();
}

vtkRay::~vtkRay() 
{
}


void vtkRay::PrintSelf(ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}
