//#include <vtkTypedArray.h>
#include <vtkDenseArray.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  //vtkSmartPointer<vtkTypedArray<double> > array = 
    //  vtkSmartPointer<vtkTypedArray<double> >::New();
  
  vtkSmartPointer<vtkDenseArray<double> > array = 
      vtkSmartPointer<vtkDenseArray<double> >::New();
  
  return 0;
}
