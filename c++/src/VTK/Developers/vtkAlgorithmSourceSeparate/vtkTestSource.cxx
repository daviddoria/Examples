#include "vtkTestSource.h"
#include "vtkTest.h"

#include "vtkDataObjectTypes.h"
#include "vtkDataSetAttributes.h"
#include "vtkDoubleArray.h"
#include "vtkGraph.h"
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include "vtkIntArray.h"
#include "vtkObjectFactory.h"
#include "vtkPointSet.h"
#include "vtkPointData.h"
#include "vtkSmartPointer.h"
#include "vtkStringArray.h"

#include <vtkstd/vector>

vtkStandardNewMacro(vtkTestSource);
vtkCxxRevisionMacro(vtkTestSource, "$Revision: 1.12 $");

vtkTestSource::vtkTestSource()
{
  //this->SetNumberOfInputPorts(0);
}

vtkTestSource::~vtkTestSource()
{
}

int vtkTestSource::FillInputPortInformation(
    int port, vtkInformation* info )
{
  if ( port == 0 )
  {
    info->Remove( vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE() );
    info->Append( vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkPointSet" );
    info->Append( vtkAlgorithm::INPUT_REQUIRED_DATA_TYPE(), "vtkGraph" );
  }
  return 1;
}

int vtkTestSource::RequestData(
                                             vtkInformation* vtkNotUsed(request),
    vtkInformationVector** inputVector,
    vtkInformationVector* outputVector )
{

  vtkIdType numPoints = 0;
  vtkInformation* inInfo  =  inputVector[0]->GetInformationObject( 0 );
  vtkDataObject* inData = inInfo->Get( vtkDataObject::DATA_OBJECT() );

  vtkGraph* graph = vtkGraph::SafeDownCast( inData );
  if ( graph )
  {
    numPoints = graph->GetNumberOfVertices();
  }

  vtkPointSet* ptset = vtkPointSet::SafeDownCast( inData );
  if ( ptset )
  {
    numPoints = ptset->GetNumberOfPoints();
  }

  vtkInformation* outInfo = outputVector->GetInformationObject( 0 );

  vtkTest* ouData = vtkTest::SafeDownCast(
      outInfo->Get( vtkDataObject::DATA_OBJECT() ) );

  return 1;
}

void vtkTestSource::PrintSelf( ostream& os, vtkIndent indent )
{
   this->Superclass::PrintSelf( os, indent );
}