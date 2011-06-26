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


void vtkRay::PrintSelf(ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
  
  //Print the rays origin and direction when << is called
  os << "Origin: " << Origin[0] << " " << Origin[1] << " " << Origin[2] << vtkstd::endl
    << "Direction: " << Direction[0] << " " << Direction[1] << " " << Direction[2] << vtkstd::endl;
	
}

void vtkRay::SetDirection(double* Dir)
{
	//set the rays direction to the unit length Dir
	vtkMath::Normalize(Dir);
  this->Direction[0] = Dir[0];
  this->Direction[1] = Dir[1];
  this->Direction[2] = Dir[2];
}
