#include <vtkSmartPointer.h>
#include <vtkDenseArray.h>
#include <vtkArrayRange.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkDenseArray<double> > array =
      vtkSmartPointer<vtkDenseArray<double> >::New();
  array->Resize(5,1);

  for(unsigned int i = 0; i < 5; i++)
    {
    array->SetValue(i, 0, 2.0);
    double val = array->GetValue(i,0);
    }

  /* old way
  int dim1start = array->GetExtent(0).GetBegin();
  int dim1end = array->GetExtent(0).GetEnd();
  cout << "dim1: " << dim1start << " to " << dim1end << endl;
  */
    vtkArrayExtents extents = array->GetExtents();
    vtkIdType dim1start = extents[0].GetBegin();
  vtkIdType test = array->GetExtents()[0].GetBegin();
    //cout << array->GetExtents()[0].GetBegin();
  //int dim1end = array->GetExtents()[0].GetEnd();
  //cout << "dim1: " << dim1start << " to " << dim1end << endl;
  /*
  int dim2start = array->GetExtent(1).GetBegin();
  int dim2end = array->GetExtent(1).GetEnd();
  cout << "dim2: " << dim2start << " to " << dim2end << endl;
  */
  return EXIT_SUCCESS;
}
