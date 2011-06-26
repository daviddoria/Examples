#include <vtkSmartPointer.h>
#include <vtkDenseArray.h>

int main(int argc, char *argv[])
{
  //create an N-D array
  vtkSmartPointer<vtkDenseArray<double> > array = 
      vtkSmartPointer<vtkDenseArray<double> >::New();
  
  //resize the array to 4x5x3
  array->Resize(4,5,3);
  
  //set a value
  int i = 0; int j = 0; int k = 0;
  double value = 3.0;
  array->SetValue(i, j, k, value);
  
  //get a value
  cout << array->GetValue(i,j,k) << endl;
  
  //get size (bounds) of whole array
  cout << array->GetExtents() << endl;
  
  //get size (bounds) of array dimensions
  cout << array->GetExtents()[0] << endl;
  cout << array->GetExtents()[1] << endl;
  cout << array->GetExtents()[2] << endl;
  
  //get the bounds of the 0th dimension
  cout << array->GetExtents()[0].GetBegin() << endl;
  cout << array->GetExtents()[0].GetEnd() << endl;
  
  return EXIT_SUCCESS;
}
