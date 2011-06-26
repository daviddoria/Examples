#include <vtkSmartPointer.h>
#include <vtkGraphToPolyData.h>
#include <vtkDataSetAttributes.h>
#include <vtkMutableUndirectedGraph.h>
#include <vtkGraphLayoutView.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkActor.h>

int main(int, char *[])
{
  // Create the first graph
  vtkSmartPointer<vtkMutableUndirectedGraph> g0 = 
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();
  {
  vtkIdType v1 = g0->AddVertex();
  vtkIdType v2 = g0->AddVertex();
  vtkIdType v3 = g0->AddVertex();
 
  g0->AddEdge(v1, v2);
  g0->AddEdge(v2, v3);
  g0->AddEdge(v1, v3);
  
  // Create points
  vtkSmartPointer<vtkPoints> points = 
    vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(1.0, 0.0, 0.0);
  points->InsertNextPoint(0.0, 1.0, 0.0);
    
  // Add the coordinates of the points to the graph
  g0->SetPoints(points);
  }
  
  // Create the second graph
  vtkSmartPointer<vtkMutableUndirectedGraph> g1 = 
    vtkSmartPointer<vtkMutableUndirectedGraph>::New();
 
  vtkIdType v1 = g1->AddVertex();
  vtkIdType v2 = g1->AddVertex();
   
  g1->AddEdge(v1, v2);
   
  // Create points
  vtkSmartPointer<vtkPoints> points = 
    vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(1.0, 0.0, 0.0);
      
  // Add the coordinates of the points to the graph
  g1->SetPoints(points);
  
  // There will be one render window
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(600, 300);
 
  // And one interactor
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = 
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);
 
  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double leftViewport[4] = {0.0, 0.0, 0.5, 1.0};
  double rightViewport[4] = {0.5, 0.0, 1.0, 1.0};
 
  // Setup both renderers
  vtkSmartPointer<vtkRenderer> leftRenderer = 
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(leftRenderer);
  leftRenderer->SetViewport(leftViewport);
  //leftRenderer->SetBackground(.6, .5, .4);  
 
  vtkSmartPointer<vtkRenderer> rightRenderer = 
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(rightRenderer);
  rightRenderer->SetViewport(rightViewport);
  //rightRenderer->SetBackground(.4, .5, .6);  
 
  // Convert the graphs to polydatas
  vtkSmartPointer<vtkGraphToPolyData> graphToPolyData0 = 
    vtkSmartPointer<vtkGraphToPolyData>::New();
  graphToPolyData0->SetInputConnection(g0->GetProducerPort());
  graphToPolyData0->Update();
  
  vtkSmartPointer<vtkGraphToPolyData> graphToPolyData1 = 
    vtkSmartPointer<vtkGraphToPolyData>::New();
  graphToPolyData1->SetInputConnection(g1->GetProducerPort());
  graphToPolyData1->Update();
  
  //Create mappers and actors
  vtkSmartPointer<vtkPolyDataMapper> mapper0 =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper0->SetInputConnection(graphToPolyData0->GetOutputPort());
  mapper0->Update();
  vtkSmartPointer<vtkActor> actor0 = 
    vtkSmartPointer<vtkActor>::New();
  actor0->SetMapper(mapper0);
  
  vtkSmartPointer<vtkPolyDataMapper> mapper1 =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper1->SetInputConnection(graphToPolyData1->GetOutputPort());
  mapper1->Update();
  vtkSmartPointer<vtkActor> actor1 = 
    vtkSmartPointer<vtkActor>::New();
  actor1->SetMapper(mapper1);
  
  // Add the sphere to the left and the cube to the right
  leftRenderer->AddActor(actor0);
  rightRenderer->AddActor(actor1);
 
  leftRenderer->ResetCamera();
 
  rightRenderer->ResetCamera();
 
  renderWindow->Render();
  interactor->Start();
  
  /*
  vtkSmartPointer<vtkGraphLayoutView> graphLayoutView = 
    vtkSmartPointer<vtkGraphLayoutView>::New();
  graphLayoutView->AddRepresentationFromInput(g0);
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();
 
  graphLayoutView->AddRepresentationFromInput(g1);
  graphLayoutView->ResetCamera();
  graphLayoutView->Render();
  graphLayoutView->GetInteractor()->Start();
  */
  return EXIT_SUCCESS;
}
