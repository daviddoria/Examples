#include "vtkTest.h"

#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkTest);

vtkTest::vtkTest()
{
  this->Value = 4.5;
}
