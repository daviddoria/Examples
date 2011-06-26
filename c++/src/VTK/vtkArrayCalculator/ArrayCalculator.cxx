#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkDoubleArray.h>
#include <vtkArrayCalculator.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkDoubleArray> array = 
      vtkSmartPointer<vtkDoubleArray>::New();
  array->SetName("MyArray");
  array->InsertNextValue(1.0);
  array->InsertNextValue(2.0);
  array->InsertNextValue(3.0);
  
  for(vtkIdType i = 0; i < array->GetNumberOfTuples(); i++)
    {
    double val = array->GetValue(i);
    cout << "input value " << i << ": " << val << endl;
    } 
  
  vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
  polydata->GetPointData()->AddArray(array);
  
  vtkSmartPointer<vtkArrayCalculator> calculator = 
      vtkSmartPointer<vtkArrayCalculator>::New();
  calculator->SetInput(polydata);
  calculator->AddScalarArrayName("MyArray");
  //calculator->SetFunction("sign(MyArray-2)");
  calculator->SetFunction("if(MyArray=2,1,0)"); //set any elements that are currently 2 to 0
  calculator->SetResultArrayName("MyArray");
  calculator->Update();
  
  vtkDoubleArray* output = vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetArray("MyArray"));
  
  for(vtkIdType i = 0; i < output->GetNumberOfTuples(); i++)
    {
    double val = output->GetValue(i);
    cout << "output value " << i << ": " << val << endl;
    } 
  return EXIT_SUCCESS;
}
