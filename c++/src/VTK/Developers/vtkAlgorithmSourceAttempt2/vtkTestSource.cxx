#include "vtkTestSource.h"

#include "vtkCallbackCommand.h"
#include "vtkDataArraySelection.h"
#include "vtkDataCompressor.h"
#include "vtkDataSet.h"
#include "vtkDataSetAttributes.h"
#include "vtkInstantiator.h"
#include "vtkObjectFactory.h"
#include "vtkXMLDataElement.h"
#include "vtkXMLDataParser.h"
#include "vtkXMLFileReadTester.h"
#include "vtkZLibDataCompressor.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkStreamingDemandDrivenPipeline.h"

#include "vtkDataSet.h"
#include "vtkPointSet.h"

vtkCxxRevisionMacro(vtkTestSource, "$Revision: 1.55 $");
vtkStandardNewMacro(vtkTestSource);

vtkTestSource::vtkTestSource()
{
  this->SetNumberOfInputPorts(0);
  this->SetNumberOfOutputPorts(1);
}

//----------------------------------------------------------------------------
vtkTestSource::~vtkTestSource()
{
  
}

//----------------------------------------------------------------------------
void vtkTestSource::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os, indent);
  
}

//----------------------------------------------------------------------------
vtkDataSet* vtkTestSource::GetOutputAsDataSet()
{
  return this->GetOutputAsDataSet(0);
}

//----------------------------------------------------------------------------
vtkDataSet* vtkTestSource::GetOutputAsDataSet(int index)
{
  return vtkDataSet::SafeDownCast( this->GetOutputDataObject(index) );
}



//----------------------------------------------------------------------------
int vtkTestSource::RequestData(vtkInformation *vtkNotUsed(request),
                              vtkInformationVector **vtkNotUsed(inputVector),
                                  vtkInformationVector *outputVector)
{
  // Get the output pipeline information and data object.
  vtkInformation* outInfo = outputVector->GetInformationObject(0);
  vtkDataObject* output = outInfo->Get(vtkDataObject::DATA_OBJECT());
  this->CurrentOutput = output;

  this->CurrentOutput = 0;
  return 1;
}


//----------------------------------------------------------------------------
void vtkTestSource::SetupOutputData()
{
  // Initialize the output.
  this->CurrentOutput->Initialize();
}

//----------------------------------------------------------------------------
vtkDataSet* vtkTestSource::GetOutput()
{
  //return this->CurrentOutput;
  return this->GetOutputAsDataSet(0);
}

//----------------------------------------------------------------------------
vtkInformation* vtkTestSource::GetCurrentOutputInformation()
{
  return this->CurrentOutputInformation;
}

int vtkTestSource::FillOutputPortInformation( int port, vtkInformation* info )
{
  /*
  if(!this->Superclass::FillOutputPortInformation(port, info))
    {
    return 0;
    }
  */
  
  if ( port == 0 )
  {
    info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkUnstructuredGrid" ); //works
    
    //info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkDataSet" ); //doesn't work "You are trying to instantiate DataObjectType "vtkDataSet" which does not exist."
    //info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkPolyData" ); //works
    
    return 1;
  }
  return 0;
}