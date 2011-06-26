#include <vtkSmartPointer.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkTransform.h>
 
int main(int argc, char *argv[])
{
  int numPoints = 10;
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(numPoints);
  pointSource->Update();
  
  for(int i = 0; i < numPoints; i++)
    {
    double point[3];
    pointSource->GetOutput()->GetPoint(i, point);
    cout << "Original point: (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << endl;
    }
 
  vtkSmartPointer<vtkTransform> translation = 
      vtkSmartPointer<vtkTransform>::New();
  translation->Translate(1.0, 2.0, 3.0);
 
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = 
      vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetInputConnection(pointSource->GetOutputPort());
  transformFilter->SetTransform(translation);
  transformFilter->Update();
 
  vtkSmartPointer<vtkPolyData> translated = 
      transformFilter->GetOutput();
 
  for(unsigned int i = 0; i < numPoints; i++)
    {
    double point[3];
    translated->GetPoint(i, point);
    cout << "Transloated point: (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << endl;
    }
 
  return EXIT_SUCCESS;
}