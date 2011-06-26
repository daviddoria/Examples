#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPoints.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPointsProjectedHull.h>
#include <vtkPolyLine.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(40);
  pointSource->Update();
  
  //setup actor and mapper
  vtkSmartPointer<vtkPolyDataMapper> pointMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  pointMapper->SetInputConnection(pointSource->GetOutputPort());
 
  vtkSmartPointer<vtkActor> pointActor = 
      vtkSmartPointer<vtkActor>::New();
  pointActor->SetMapper(pointMapper);
  
  //why don't these work?
  //vtkPointsProjectedHull* pointsHull = dynamic_cast<vtkPointsProjectedHull*>(pointSource->GetOutput());
  //vtkPointsProjectedHull* pointsHull = vtkPointsProjectedHull::SafeDownCast(pointSource->GetOutput());
  vtkSmartPointer<vtkPointsProjectedHull> points = 
      vtkSmartPointer<vtkPointsProjectedHull>::New();
  points->DeepCopy(pointSource->GetOutput()->GetPoints());
  
  int xSize = points->GetSizeCCWHullX();
  cout << "xSize: " << xSize << endl;
  
  double* pts = new double[xSize*2];
  
  points->GetCCWHullX(pts,xSize);
  
  vtkSmartPointer<vtkPoints> xHullPoints = 
      vtkSmartPointer<vtkPoints>::New();
  for(unsigned int i = 0; i < xSize; i++)
    {
    double yval = pts[2*i];
    double zval = pts[2*i + 1];
    cout << "(y,z) value of point " << i << " : (" << yval << " , " << zval << ")" << endl;
    xHullPoints->InsertNextPoint(0.0, yval, zval);
    }
    //insert the first point again to close the loop
  xHullPoints->InsertNextPoint(0.0, pts[0], pts[1]);
    
  //display the x hull
  vtkSmartPointer<vtkPolyLine> xPolyLine = 
      vtkSmartPointer<vtkPolyLine>::New();
  xPolyLine->GetPointIds()->SetNumberOfIds(xHullPoints->GetNumberOfPoints());
  
  for(unsigned int i = 0; i < xHullPoints->GetNumberOfPoints(); i++)
    {
    xPolyLine->GetPointIds()->SetId(i,i);
    }
    
  //Create a cell array to store the lines in and add the lines to it
  vtkSmartPointer<vtkCellArray> cells = 
      vtkSmartPointer<vtkCellArray>::New();
  cells->InsertNextCell(xPolyLine);

  //Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = 
      vtkSmartPointer<vtkPolyData>::New();

  //add the points to the dataset
  polyData->SetPoints(xHullPoints);

  //add the lines to the dataset
  polyData->SetLines(cells);
  
  //setup actor and mapper
  vtkSmartPointer<vtkPolyDataMapper> xHullMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  xHullMapper->SetInput(polyData);
 
  vtkSmartPointer<vtkActor> xHullActor = 
      vtkSmartPointer<vtkActor>::New();
  xHullActor->SetMapper(xHullMapper);
 
  //setup render window, renderer, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
  
  renderer->AddActor(xHullActor);
  renderer->AddActor(pointActor);
 
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
