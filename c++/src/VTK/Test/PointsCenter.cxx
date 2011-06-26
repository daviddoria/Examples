#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

void test1();
void test2();

int main(int argc, char *argv[])
{
      
  test1();
  test2();
  return EXIT_SUCCESS;
}

void test1()
{
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0,0,0);
  points->InsertNextPoint(1,0,0);
  points->InsertNextPoint(0,1,0);
  points->InsertNextPoint(1,1,0);
  
  double center[3];
  points->CenterOfMass(center);
  cout << "Center: " << center[0] << " " << center[1] << " " << center[2] << endl;
  
}

void test2()
{
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0,0,0);
  points->InsertNextPoint(1,0,0);
  points->InsertNextPoint(0,1,0);
  points->InsertNextPoint(1,1,0);
  
  vtkSmartPointer<vtkPolyData> pd = 
      vtkSmartPointer<vtkPolyData>::New();
  pd->SetPoints(points);
  
  double center[3];
  pd->GetPoints()->CenterOfMass(center);
  cout << "Center: " << center[0] << " " << center[1] << " " << center[2] << endl;
  
}