#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

void OutputPoints(vtkSmartPointer<vtkPoints> Points);
void ReallyDeletePoint(vtkSmartPointer<vtkPoints> points, unsigned int id);

int main(int argc, char *argv[])
{
  //create a set of points
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  points->InsertNextPoint ( 0.0, 1.0, 0.0 );
  points->InsertNextPoint ( 0.0, 0.0, 1.0 );

  cout << "Number of points: " << points->GetNumberOfPoints() << endl;
  OutputPoints(points);
  
  cout << "Number of points: " << points->GetNumberOfPoints() << endl;
  OutputPoints(points);
  
  return EXIT_SUCCESS;
}

void OutputPoints(vtkSmartPointer<vtkPoints> points)
{
  for(unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
    {
    double p[3];
    points->GetPoint(i,p);
    cout << p[0] << " " << p[1] << " " << p[2] << endl;
    }
}

void ReallyDeletePoint(vtkSmartPointer<vtkPoints> points, unsigned int id)
{
  vtkSmartPointer<vtkPoints> newPoints = 
      vtkSmartPointer<vtkPoints>::New();
  
  for(unsigned int i = 0; i < points->GetNumberOfPoints(); i++)
    {
    if(i != id)
      {
      double p[3];
      points->GetPoint(i,p);
      newPoints->InsertNextPoint(p);
      }
    }
  
  points->ShallowCopy(newPoints);
}