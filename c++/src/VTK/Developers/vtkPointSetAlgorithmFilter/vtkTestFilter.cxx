#include "vtkTestFilter.h"

#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkDataObject.h"
#include "vtkSmartPointer.h"
#include "vtkPointSet.h"

vtkCxxRevisionMacro(vtkTestFilter, "$Revision: 1.70 $");
vtkStandardNewMacro(vtkTestFilter);

int vtkTestFilter::RequestData(vtkInformation *vtkNotUsed(request),
                                             vtkInformationVector **inputVector,
                                             vtkInformationVector *outputVector)
{
  
  // get the info objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  
  
  // get the input and ouptut
  vtkPointSet* input = vtkPointSet::SafeDownCast(
      inInfo->Get(vtkDataObject::DATA_OBJECT()));
  
  vtkPointSet* output = vtkPointSet::SafeDownCast(
      outInfo->Get(vtkDataObject::DATA_OBJECT()));
  
  cout << "output is a : " << output->GetClassName() << endl; 
  
  output->ShallowCopy(input);
    
  return 1;
}


//----------------------------------------------------------------------------
void vtkTestFilter::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}

