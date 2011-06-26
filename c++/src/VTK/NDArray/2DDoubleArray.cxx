#include <vtkSmartPointer.h>
#include <vtkDenseArray.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkDenseArray<double> > array = 
      vtkSmartPointer<vtkDenseArray<double> >::New();
  array->Resize(2,5);
  
  array->SetValue(0,4, 5.0);
  
  cout << array->GetValue(0,4) << endl;
  
  cout << array->GetExtents() << endl;
  
  cout << array->GetExtents()[0] << endl;
  cout << array->GetExtents()[0].GetEnd() << endl;
  
  return EXIT_SUCCESS;
}
