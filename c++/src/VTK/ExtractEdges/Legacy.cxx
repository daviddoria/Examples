#include <vtkPolyData.h>
#include <vtkMath.h>
#include <vtkExtractEdges.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkDelaunay3D.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  vtkSmartPointer<vtkExtractEdges> extractEdges = 
      vtkSmartPointer<vtkExtractEdges>::New();
  extractEdges->SetInputConnection(sphereSource->GetOutputPort());
  extractEdges->Update();
  
  vtkCellArray* lines= extractEdges->GetOutput()->GetLines();
  vtkPoints* points = extractEdges->GetOutput()->GetPoints();

  cout << "There are " << lines->GetNumberOfCells() << " line cells." << endl;

  cout << "Computing angles... " << endl;
  double angle;
  
  vtkIdType npts, *pts;
  double p0[3], p1[3];

  lines->InitTraversal();
  
  while(lines->GetNextCell(npts, pts))
  {
    if (npts != 2)
    {
      cout << "Didn't get two points! Skipping edge!" << endl;
      continue;
    }
    epoints->GetPoint(pts[0], p0);
    epoints->GetPoint(pts[1], p1);
    vtkMath::Normalize(p0);
    vtkMath::Normalize(p1);
    angle = acos(vtkMath::Dot(p0, p1)) * 180.0 / vtkMath::Pi();
    cout << angle << "; ";
  }
  
  cout << endl;

  return 0;
}
