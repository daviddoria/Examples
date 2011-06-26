#include <vtkSmartPointer.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkAppendPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

int main(int, char *[])
{
  // Create one set of poitns.
  vtkSmartPointer<vtkPointSource> pointSource1 =
      vtkSmartPointer<vtkPointSource>::New();
  pointSource1->SetNumberOfPoints(5);
  pointSource1->Update();
  cout << "pointSource1 has " << pointSource1->GetOutput()->GetNumberOfPoints() << " points and "
       << pointSource1->GetOutput()->GetNumberOfPoints() << " cells and "
       << pointSource1->GetOutput()->GetNumberOfVerts() <<  " verts." << endl;

  // Create another set of poitns.
  vtkSmartPointer<vtkPointSource> pointSource2 =
    vtkSmartPointer<vtkPointSource>::New();
  pointSource2->SetNumberOfPoints(5);
  pointSource2->Update();
  cout << "pointSource2 has "
       << pointSource2->GetOutput()->GetNumberOfPoints() << " points and "
       << pointSource2->GetOutput()->GetNumberOfPoints() << " cells and "
       << pointSource2->GetOutput()->GetNumberOfVerts() <<  " verts." << endl;

  // Combine the two point sets
  vtkSmartPointer<vtkAppendPoints> appendFilter =
      vtkSmartPointer<vtkAppendPoints>::New();
  appendFilter->AddInputConnection(pointSource1->GetOutputPort());
  appendFilter->AddInputConnection(pointSource2->GetOutputPort());
  appendFilter->Update();

  cout << "Combined there are "
       << appendFilter->GetOutput()->GetNumberOfPoints() << " points and "
       << appendFilter->GetOutput()->GetNumberOfPoints() << " cells and "
       << appendFilter->GetOutput()->GetNumberOfVerts() <<  " verts." << endl;

  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(appendFilter->GetOutputPort());
  //mapper->SetInputConnection(pointSource1->GetOutputPort());

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
  //renderer->SetBackground(.3, .6, .3); // Background color green

  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}