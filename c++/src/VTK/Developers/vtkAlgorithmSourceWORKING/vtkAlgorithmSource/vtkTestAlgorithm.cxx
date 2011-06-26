#include "vtkTestAlgorithm.h"

#include "vtkCommand.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkObjectFactory.h"
#include "vtkTest.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkTrivialProducer.h"

vtkCxxRevisionMacro(vtkTestAlgorithm, "$Revision: 1.1 $");
vtkStandardNewMacro(vtkTestAlgorithm);

//----------------------------------------------------------------------------
vtkTestAlgorithm::vtkTestAlgorithm()
{
  // by default assume filters have one input and one output
  // subclasses that deviate should modify this setting
  this->SetNumberOfInputPorts( 1 );
  this->SetNumberOfOutputPorts( 1 );
}

//----------------------------------------------------------------------------
vtkTestAlgorithm::~vtkTestAlgorithm()
{
}

//----------------------------------------------------------------------------
void vtkTestAlgorithm::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
}

//----------------------------------------------------------------------------
vtkTest* vtkTestAlgorithm::GetOutput()
{
  return this->GetOutput(0);
}

//----------------------------------------------------------------------------
vtkTest* vtkTestAlgorithm::GetOutput(int port)
{
  return vtkTest::SafeDownCast(this->GetOutputDataObject(port));
}

//----------------------------------------------------------------------------
void vtkTestAlgorithm::SetOutput(vtkDataObject* d)
{
  this->GetExecutive()->SetOutputData(0, d);
}

//----------------------------------------------------------------------------
vtkDataObject* vtkTestAlgorithm::GetInput()
{
  return this->GetInput(0);
}

//----------------------------------------------------------------------------
vtkDataObject* vtkTestAlgorithm::GetInput(int port)
{
  return this->GetExecutive()->GetInputData(port, 0);
}

//----------------------------------------------------------------------------
vtkTest* vtkTestAlgorithm::GetLabelHierarchyInput(int port)
{
  return vtkTest::SafeDownCast(this->GetInput(port));
}

//----------------------------------------------------------------------------
int vtkTestAlgorithm::ProcessRequest(vtkInformation* request,
                                               vtkInformationVector** inputVector,
                                               vtkInformationVector* outputVector)
{
  // Create an output object of the correct type.
  if(request->Has(vtkDemandDrivenPipeline::REQUEST_DATA_OBJECT()))
  {
    return this->RequestDataObject(request, inputVector, outputVector);
  }
  // generate the data
  if(request->Has(vtkDemandDrivenPipeline::REQUEST_DATA()))
  {
    return this->RequestData(request, inputVector, outputVector);
  }

  if(request->Has(vtkStreamingDemandDrivenPipeline::REQUEST_UPDATE_EXTENT()))
  {
    return this->RequestUpdateExtent(request, inputVector, outputVector);
  }

  // execute information
  if(request->Has(vtkDemandDrivenPipeline::REQUEST_INFORMATION()))
  {
    return this->RequestInformation(request, inputVector, outputVector);
  }

  return this->Superclass::ProcessRequest(request, inputVector, outputVector);
}

//----------------------------------------------------------------------------
int vtkTestAlgorithm::FillOutputPortInformation(
    int vtkNotUsed(port), vtkInformation* info)
{
  // now add our info
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkTest");
  return 1;
}

//----------------------------------------------------------------------------
int vtkTestAlgorithm::FillInputPortInformation(
    int vtkNotUsed(port), vtkInformation* info)
{
  info->Set(vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkTest");
  return 1;
}

//----------------------------------------------------------------------------
int vtkTestAlgorithm::RequestDataObject(
    vtkInformation* vtkNotUsed(request),
                               vtkInformationVector** vtkNotUsed(inputVector),
                                   vtkInformationVector* outputVector )
{
  for ( int i = 0; i < this->GetNumberOfOutputPorts(); ++i )
  {
    vtkInformation* outInfo = outputVector->GetInformationObject( i );
    vtkTest* output = vtkTest::SafeDownCast(
        outInfo->Get( vtkDataObject::DATA_OBJECT() ) );
    if ( ! output )
    {
      output = vtkTest::New();
      outInfo->Set( vtkDataObject::DATA_OBJECT(), output );
      output->FastDelete();
      output->SetPipelineInformation( outInfo );
      this->GetOutputPortInformation( i )->Set(
                                      vtkDataObject::DATA_EXTENT_TYPE(), output->GetExtentType() );
    }
  }
  return 1;
}

//----------------------------------------------------------------------------
int vtkTestAlgorithm::RequestInformation(
    vtkInformation* vtkNotUsed(request),
                               vtkInformationVector** vtkNotUsed(inputVector),
                                   vtkInformationVector* vtkNotUsed(outputVector))
{
  // do nothing let subclasses handle it
  return 1;
}

//----------------------------------------------------------------------------
int vtkTestAlgorithm::RequestUpdateExtent(
    vtkInformation* vtkNotUsed(request),
                               vtkInformationVector** inputVector,
                               vtkInformationVector* vtkNotUsed(outputVector))
{
  int numInputPorts = this->GetNumberOfInputPorts();
  for (int i=0; i<numInputPorts; i++)
  {
    int numInputConnections = this->GetNumberOfInputConnections(i);
    for (int j=0; j<numInputConnections; j++)
    {
      vtkInformation* inputInfo = inputVector[i]->GetInformationObject(j);
      inputInfo->Set(vtkStreamingDemandDrivenPipeline::EXACT_EXTENT(), 1);
    }
  }
  return 1;
}

//----------------------------------------------------------------------------
// This is the superclasses style of Execute method.  Convert it into
// an imaging style Execute method.
int vtkTestAlgorithm::RequestData(
                                            vtkInformation* vtkNotUsed(request),
    vtkInformationVector** vtkNotUsed( inputVector ),
                                       vtkInformationVector* vtkNotUsed(outputVector) )
{
  // do nothing let subclasses handle it
  return 1;
}

//----------------------------------------------------------------------------
void vtkTestAlgorithm::SetInput(vtkDataObject* input)
{
  this->SetInput(0, input);
}

//----------------------------------------------------------------------------
void vtkTestAlgorithm::SetInput(int index, vtkDataObject* input)
{
  if(input)
  {
    this->SetInputConnection(index, input->GetProducerPort());
  }
  else
  {
    // Setting a NULL input removes the connection.
    this->SetInputConnection(index, 0);
  }
}

//----------------------------------------------------------------------------
void vtkTestAlgorithm::AddInput(vtkDataObject* input)
{
  this->AddInput(0, input);
}

//----------------------------------------------------------------------------
void vtkTestAlgorithm::AddInput(int index, vtkDataObject* input)
{
  if(input)
  {
    this->AddInputConnection(index, input->GetProducerPort());
  }
}
