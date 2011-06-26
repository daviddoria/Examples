#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>

void PrintTriangle(vtkTriangle* Triangle);

int main(int argc, char *argv[])
{
  
  vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPoints()->SetPoint(0, 1.0, 0.0, 0.0);
  triangle->GetPoints()->SetPoint(1, 0.0, 0.0, 0.0);
  triangle->GetPoints()->SetPoint(2, 0.0, 1.0, 0.0);
  
  PrintTriangle(triangle);
  
  return 0;
}

void PrintTriangle(vtkTriangle* Triangle)
{
  double p[3];
  for(unsigned int i = 0; i < 3; i++)
  {
    Triangle->GetPoints()->GetPoint(i, p);
    vtkstd::cout << p[0] << " " << p[1] << " " << p[2] << vtkstd::endl;
  }
}