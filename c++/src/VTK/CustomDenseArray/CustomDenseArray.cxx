#include <vtkSmartPointer.h>
#include <vtkDenseArray.h>

/*
struct Point
{
  double x,y;
  
  //operator vtkVariant() const { return vtkVariant(); }
};
*/


class Point
{
  public:
  double x,y;
  
  operator vtkVariant() const { return vtkVariant(); }
};

int main(int argc, char *argv[])
{
  Point MyPoint;
  MyPoint.x = 1.0;
  MyPoint.y = 2.0;
  
  vtkstd::cout << MyPoint.x << " " << MyPoint.y << vtkstd::endl;
      
  vtkSmartPointer<vtkDenseArray<Point> > array = vtkSmartPointer<vtkDenseArray<Point> >::New();
  array->Resize(5,5);
  
  array->SetValue(4,4, MyPoint);
  
  Point RetrievedPoint = array->GetValue(4,4);
  vtkstd::cout << RetrievedPoint.x << " " << RetrievedPoint.y << vtkstd::endl;
  return 0;
}
