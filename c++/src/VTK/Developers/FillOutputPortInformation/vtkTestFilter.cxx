#include "vtkTestFilter.h"

#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"
#include "vtkDataObject.h"
#include "vtkSmartPointer.h"
#include "vtkMutableUndirectedGraph.h"
#include "vtkMutableDirectedGraph.h"
#include "vtkMutableGraphHelper.h"

#include <string>

vtkCxxRevisionMacro(vtkTestFilter, "$Revision: 1.70 $");
vtkStandardNewMacro(vtkTestFilter);

vtkTestFilter::vtkTestFilter()
{
  this->SetNumberOfInputPorts(1);
  this->SetNumberOfOutputPorts(1);
}

int vtkTestFilter::FillInputPortInformation(int port, vtkInformation *info)
{
  info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkGraph" );
}

int vtkTestFilter::FillOutputPortInformation( int port, vtkInformation* info )
{
  //std::cout << "OutputType is " << this->OutputType << std::endl;
  
    if(this->OutputType.compare("ug") == 0)
     {
     std::cout << "Setting output to ug" << std::endl;
     info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkUndirectedGraph");
     }
    else if(this->OutputType.compare("dg") == 0)
     {
     std::cout << "Setting output to dg" << std::endl;
     info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkDirectedGraph");
     }
    else if(this->OutputType.compare("mdg") == 0)
     {
     std::cout << "Setting output to mdg" << std::endl;
     info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkMutableDirectedGraph");
     }
    else if(this->OutputType.compare("mug") == 0)
     {
     std::cout << "Setting output to mug" << std::endl;
     info->Set(vtkDataObject::DATA_TYPE_NAME(), "vtkMutableUndirectedGraph");
     }
    else
     {
     std::cout << "Invalid output type specified." << std::endl;
     }
} 

/*
int vtkTestFilter::ProcessRequest(vtkInformation* request,
                                            vtkInformationVector** inInfoVec,
                                            vtkInformationVector* outInfoVec)
{
  if(request->Has(vtkDemandDrivenPipeline::REQUEST_DATA_OBJECT()))
    {
    return vtkAlgorithm::ProcessRequest(request, inInfoVec, outInfoVec);
    }

  return this->Superclass::ProcessRequest(request, inInfoVec, outInfoVec);
}
*/

int vtkTestFilter::RequestData(vtkInformation *vtkNotUsed(request),
                                             vtkInformationVector **inputVector,
                                             vtkInformationVector *outputVector)
{
  
  // get the info objects
  vtkInformation *inInfo = inputVector[0]->GetInformationObject(0);
  vtkInformation *outInfo = outputVector->GetInformationObject(0);
    
  // get the input and ouptut
  vtkGraph *input = vtkGraph::SafeDownCast(
      inInfo->Get(vtkDataObject::DATA_OBJECT()));

  
  vtkGraph* output = vtkGraph::SafeDownCast(
      outInfo->Get(vtkDataObject::DATA_OBJECT()));

  std::cout << "Output is type: " << output->GetClassName() << std::endl;
  
  return 1;
}

