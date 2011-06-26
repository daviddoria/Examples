#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkExtractEdges.h>
#include <vtkSphereSource.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkLine.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  cout << "Sphere" << endl << "----------" << endl;
  cout << "There are " << sphereSource->GetOutput()->GetNumberOfCells() << " cells." << endl;
  cout << "There are " << sphereSource->GetOutput()->GetNumberOfPoints() << " points." << endl;
  
  vtkSmartPointer<vtkExtractEdges> extractEdges = 
      vtkSmartPointer<vtkExtractEdges>::New();
  extractEdges->SetInputConnection(sphereSource->GetOutputPort());
  extractEdges->Update();
  
  vtkCellArray* lines= extractEdges->GetOutput()->GetLines();
  vtkPoints* points = extractEdges->GetOutput()->GetPoints();

  cout << endl << "Edges" << endl << "----------" << endl;
  cout << "There are " << lines->GetNumberOfCells() << " cells." << endl;
  cout << "There are " << points->GetNumberOfPoints() << " points." << endl;

  //traverse all of the edges
  for(vtkIdType i = 0; i < extractEdges->GetOutput()->GetNumberOfCells(); i++)
    {
    //cout << "Type: " << extractEdges->GetOutput()->GetCell(i)->GetClassName() << endl;
    vtkSmartPointer<vtkLine> line = vtkLine::SafeDownCast(extractEdges->GetOutput()->GetCell(i));
    cout << "Line " << i << " : " << *line << endl;
    }
  
  //visualize the edges
  
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(extractEdges->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
 
  //Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);
 
  //Add the actor to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white
 
  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
