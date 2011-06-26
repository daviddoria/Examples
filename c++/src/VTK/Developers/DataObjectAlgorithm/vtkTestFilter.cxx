#include "vtkTestFilter.h"

#include "vtkCommand.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"

vtkCxxRevisionMacro(vtkTestFilter, "$Revision: 1.1 $");
vtkStandardNewMacro(vtkTestFilter);

//----------------------------------------------------------------------------
vtkTestFilter::vtkTestFilter()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}


//----------------------------------------------------------------------------
int vtkTestFilter::FillOutputPortInformation(
    int vtkNotUsed(port), vtkInformation* info)
{
  // now add our info
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkUnstructuredGrid");
  return 1;
}

//----------------------------------------------------------------------------
int vtkTestFilter::FillInputPortInformation(
                                               int vtkNotUsed(port), vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPolyData");
  return 1;
}


    int vtkTestFilter::RequestDataObject(vtkInformation* vtkNotUsed(request),
                                         vtkInformationVector** vtkNotUsed(inputVector),
         vtkInformationVector* outputVector )
{
//RequestDataObject (RDO) is an earlier pipeline pass.
//During RDO, each filter is supposed to produce an empty data object of the proper type
/*
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  vtkTest* output = vtkTest::SafeDownCast(
                                            outInfo->Get( vtkDataObject::DATA_OBJECT() ) );
  
  if ( ! output )
    {
    output = vtkTest::New();
    outInfo->Set( vtkDataObject::DATA_OBJECT(), output );
    output->FastDelete();
    output->SetPipelineInformation( outInfo );
    
    this->GetOutputPortInformation(0)->Set(
                                    vtkDataObject::DATA_EXTENT_TYPE(), output->GetExtentType() );
    }
    */

  return 1;
}


int vtkTestFilter::RequestInformation(
                                         vtkInformation* vtkNotUsed(request),
    vtkInformationVector** vtkNotUsed(inputVector),
                                      vtkInformationVector* vtkNotUsed(outputVector))
{
  // do nothing let subclasses handle it
  return 1;
}


int vtkTestFilter::RequestData(
                                  vtkInformation* vtkNotUsed(request),
    vtkInformationVector **inputVector,
    vtkInformationVector* outputVector )
{
  //Later on RequestData (RD) happens.
//During RD each filter examines any inputs it has, then fills in that empty data object with real data.

/*
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  vtkTest* output = vtkTest::SafeDownCast(
                                          outInfo->Get( vtkDataObject::DATA_OBJECT() ) );
    
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkTest *input = vtkTest::SafeDownCast(
                                         inInfo->Get(vtkDataObject::DATA_OBJECT()));
  output->ShallowCopy(input);
  
  */  
  return 1;
}
