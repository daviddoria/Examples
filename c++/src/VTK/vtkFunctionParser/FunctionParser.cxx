#include <vtkSmartPointer.h>
#include <vtkFunctionParser.h>


int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkFunctionParser> functionParser = 
      vtkSmartPointer<vtkFunctionParser>::New();
  functionParser->SetFunction("a+b");
  
  functionParser->SetScalarVariableValue( "a", 2);
  functionParser->SetScalarVariableValue( "b", 3);
    
  double result = functionParser->GetScalarResult();
  
  cout << "result: " << result << endl;
  
  return 0;
}
