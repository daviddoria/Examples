#include <vtkSmartPointer.h>
#include <vtkDataObject.h>
#include <vtkObjectFactory.h>

class TestClass : public vtkDataObject
{
  public:
  static TestClass *New();
  vtkTypeRevisionMacro(TestClass,vtkDataObject);
  TestClass()
  {
    vtkWarningMacro("Test warning.");
  }
};

vtkCxxRevisionMacro(TestClass, "$Revision: 1.1 $");
vtkStandardNewMacro(TestClass);

int main(int argc, char *argv[])
{

  vtkSmartPointer<TestClass> test =
      vtkSmartPointer<TestClass>::New();

  return EXIT_SUCCESS;
}
