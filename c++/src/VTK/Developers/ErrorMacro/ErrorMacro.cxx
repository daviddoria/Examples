#include <vtkSmartPointer.h>
#include <vtkDataObject.h>
#include <vtkObjectFactory.h>

class TestClass : public vtkDataObject
{
  public:
  static TestClass *New();
  vtkTypeMacro(TestClass,vtkDataObject);
  TestClass()
  {
   
  }

  void MemberFunction()
  {
    vtkErrorMacro("Test error.");
  }

  static void StaticFunction()
  {
    vtkGenericWarningMacro("Static error.");
  }
};

vtkStandardNewMacro(TestClass);

int main(int argc, char *argv[])
{

  vtkSmartPointer<TestClass> test =
    vtkSmartPointer<TestClass>::New();

  test->MemberFunction();

  TestClass::StaticFunction();
  
  return EXIT_SUCCESS;
}
