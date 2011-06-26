#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSphereSource.h>
#include <vtkSmartPointer.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkIntArray.h>
#include <vtkDataArray.h>
 
int main(int argc, char **argv)
{ 
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(0.0, 0.0, 0.0);
  sphereSource->SetRadius(1.0);
  
  vtkPolyData* sphere = sphereSource->GetOutput();
  
  double testInside[3] = {0.0, 0.0, 0.0};
  double testOutside[3] = {10.0, 0.0, 0.0};
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(testInside);
  points->InsertNextPoint(testOutside);
  
  vtkSmartPointer<vtkPolyData> pointsPolydata = 
      vtkSmartPointer<vtkPolyData>::New();
  pointsPolydata->SetPoints(points);
  
  vtkSmartPointer<vtkSelectEnclosedPoints> selectEnclosedPoints = 
      vtkSmartPointer<vtkSelectEnclosedPoints>::New();
  selectEnclosedPoints->SetInput(pointsPolydata);
  selectEnclosedPoints->SetSurface(sphere);
  selectEnclosedPoints->Update();
  
  for(unsigned int i = 0; i < 2; i++)
    {
    cout << "Point " << i << ": " << selectEnclosedPoints->IsInside(i) << endl;
    }
  
  vtkDataArray* insideArray = vtkDataArray::SafeDownCast(selectEnclosedPoints->GetOutput()->GetPointData()->GetArray("SelectedPoints"));
  
  for(unsigned int i = 0; i < insideArray->GetNumberOfTuples(); i++)
    { 
    cout << i << " : " << insideArray->GetComponent(i,0) << endl;
    }
  
  return EXIT_SUCCESS;
}
