#include "vtkTestSource.h"
#include "vtkTest.h"

#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkDataObject.h"

vtkCxxRevisionMacro(vtkTestSource, "$Revision: 1.70 $");
vtkStandardNewMacro(vtkTestSource);

vtkTestSource::vtkTestSource()
{
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

vtkTestSource::~vtkTestSource()
{

}

vtkTest* vtkTestSource::GetOutput()
{
  return vtkTest::SafeDownCast(this->GetOutputDataObject(0));
}

int vtkTestSource::FillOutputPortInformation( int port, vtkInformation* info )
{
  if ( port == 0 )
  {
    info->Set(vtkObject::DATA_TYPE_NAME(), "vtkTest" );
    return 1;
  }

  return 0;
}

int vtkTestSource::RequestData(
  vtkInformation *vtkNotUsed(request),
  vtkInformationVector **vtkNotUsed(inputVector),
  vtkInformationVector *outputVector)
{
  
  // get the info object
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
  
  // get the ouptut
   vtkTest *output = vtkTest::SafeDownCast(
                    outInfo->Get(vtkObject::DATA_OBJECT()));
  
  output = vtkTest::New();
  output->SetValue(5.1) ;
  
  return 1;
}


//----------------------------------------------------------------------------
void vtkTestSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}

