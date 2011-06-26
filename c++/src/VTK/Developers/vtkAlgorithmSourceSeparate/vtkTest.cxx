#include "vtkTest.h"

#include "vtkCellType.h"
#include "vtkDataArray.h"
#include "vtkIdTypeArray.h"
#include "vtkIdList.h"
#include "vtkIntArray.h"
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkTest);
vtkCxxRevisionMacro(vtkTest,"$Revision: 1.46 $");

vtkTest::vtkTest()
{
  this->Value = 4.5;
}

vtkTest::~vtkTest()
{

}

void vtkTest::PrintSelf( ostream& os, vtkIndent indent )
{
  this->Superclass::PrintSelf( os, indent );
}
