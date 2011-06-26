#include "vtkTest.h"

#include "vtkObjectFactory.h"
#include "vtkStreamingDemandDrivenPipeline.h"
#include "vtkInformationVector.h"
#include "vtkInformation.h"

vtkCxxRevisionMacro(vtkTest, "$Revision: 1.70 $");
vtkStandardNewMacro(vtkTest);

vtkTest::vtkTest()
{
  
}

vtkTest::~vtkTest()
{

}

void vtkTest::PrintSelf(ostream& os, vtkIndent indent)
{
  this->Superclass::PrintSelf(os,indent);
  os << indent << "Value: " << this->Value << "\n";	
}

/*
void vtkTest::ShallowCopy(vtkTest* t)
{
  if(!this)
    {
      this = vtkTest::New();
    }
    
  this->SetValue(t->GetValue());
}
*/