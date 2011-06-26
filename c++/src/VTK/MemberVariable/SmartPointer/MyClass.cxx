#include "vtkObjectFactory.h" //for new() macro
#include "vtkInformation.h"
#include "vtkInformationVector.h"

#include "vtkDenseArray.h"
#include "vtkSmartPointer.h"

#include "MyClass.h"

vtkCxxRevisionMacro(vtkMyClass, "$Revision: 1.1 $");
vtkStandardNewMacro(vtkMyClass);

vtkMyClass::vtkMyClass()
{
	
}

vtkMyClass::~vtkMyClass()
{

}

int vtkMyClass::RequestData(vtkInformation *vtkNotUsed(request),
		vtkInformationVector **inputVector,
  vtkInformationVector *outputVector)
{

	
	return 1;
}


void vtkMyClass::PrintSelf(ostream &os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
  
}