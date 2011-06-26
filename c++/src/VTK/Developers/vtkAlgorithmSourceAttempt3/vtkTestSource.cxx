#include "vtkTestSource.h"

#include "vtkSmartPointer.h"
#include "vtkCommand.h"
#include "vtkFloatArray.h"
#include "vtkImageData.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkMapper.h"
#include "vtkObjectFactory.h"
#include "vtkPointData.h"
#include "vtkRenderWindow.h"
#include "vtkRenderer.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkUnsignedCharArray.h"

vtkCxxRevisionMacro(vtkTestSource, "$Revision: 1.63 $");
vtkStandardNewMacro(vtkTestSource);


vtkTestSource::vtkTestSource()
{
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}


vtkTestSource::~vtkTestSource()
{
  
}

void vtkTestSource::RequestData(vtkInformation*,
                                    vtkInformationVector**,
                                    vtkInformationVector* outputVector)
{
  vtkInformation* info = outputVector->GetInformationObject(0);
  
  //vtkImageData *output = 
    //  vtkImageData::SafeDownCast(info->Get(vtkDataObject::DATA_OBJECT()));
  
  vtkTest *output = 
      vtkTest::SafeDownCast(info->Get(vtkDataObject::DATA_OBJECT()));
  
  //vtkDataObject *output = 
    //  vtkDataObject::SafeDownCast(info->Get(vtkDataObject::DATA_OBJECT()));
  
  if(!output)
  {
    output = vtkTest::New();
    output->SetValue(5.3);
    //info->Set(vtkDataObject::DATA_OBJECT(), output);
    info->Set(vtkDataObject::DATA_OBJECT(), vtkDataObject::SafeDownCast(output));
  }
  
  
  //vtkstd::cout << output->GetClassName() << vtkstd::endl;
  
  
  //vtkTest* NewTest = vtkTest::New();
  //NewTest->SetValue(5.3);
  
  //output->ShallowCopy(NewTest);
  
  //vtkstd::cout << *output << vtkstd::endl;
  
}

void vtkTestSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
}


//----------------------------------------------------------------------------
void vtkTestSource::RequestInformation (
                                            vtkInformation * vtkNotUsed(request),
    vtkInformationVector** vtkNotUsed( inputVector ),
                                       vtkInformationVector *outputVector)
{
  
  //vtkInformation* outInfo = outputVector->GetInformationObject(0);

//  outInfo->Set(vtkDataObject::DATA_OBJECT(), extent, 6);
  //outInfo->Set(vtkStreamingDemandDrivenPipeline::WHOLE_EXTENT(), extent, 6);

  //vtkDataObject::SetPointDataActiveScalarInfo(outInfo, VTK_UNSIGNED_CHAR, 
    //  3 + (this->DepthValuesInScalars ? 1:0));
  
}

//----------------------------------------------------------------------------
int vtkTestSource::ProcessRequest(vtkInformation* request,
                                      vtkInformationVector** inputVector,
                                      vtkInformationVector* outputVector)
{
  // generate the data
  if(request->Has(vtkDemandDrivenPipeline::REQUEST_DATA()))
  {
    this->RequestData(request, inputVector, outputVector);
    return 1;
  }

  // execute information
  if(request->Has(vtkDemandDrivenPipeline::REQUEST_INFORMATION()))
  {
    this->RequestInformation(request, inputVector, outputVector);
    return 1;
  }

  return this->Superclass::ProcessRequest(request, inputVector, outputVector);
}

vtkTest* vtkTestSource::GetOutput()
{
  return vtkTest::SafeDownCast(this->GetOutputDataObject(0));
}


int vtkTestSource::FillOutputPortInformation(
    int vtkNotUsed(port), vtkInformation* info)
{
  // now add our info
  //info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkImageData");
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkTest");
  //NewDataObject(): You are trying to instantiate DataObjectType "vtkTest" which does not exist.

  return 1;
}
