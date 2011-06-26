#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkCharArray.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkLabeledDataMapper.h>
#include <vtkActor.h>
#include <vtkStringArray.h>
#include <vtkActor2D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

int main(int argc, char *argv[])
{
  
  //Create a point set
  vtkSmartPointer<vtkPointSource> pointSource = 
      vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetNumberOfPoints(3);
  pointSource->Update();

  vtkSmartPointer<vtkPolyData> points = pointSource->GetOutput();
  
  //Create a mapper and actor
  
  vtkSmartPointer<vtkPolyDataMapper> pointMapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  pointMapper->SetInputConnection(pointSource->GetOutputPort());
  
  vtkSmartPointer<vtkActor> pointActor = 
      vtkSmartPointer<vtkActor>::New();
  pointActor->SetMapper(pointMapper);

  
  vtkSmartPointer<vtkStringArray> strings = 
      vtkSmartPointer<vtkStringArray>::New();
  strings->SetName("strings");
  strings->InsertNextValue("one");
  strings->InsertNextValue("two");
  strings->InsertNextValue("three");
  
  /*
  vtkSmartPointer<vtkCharArray> strings = 
    vtkSmartPointer<vtkCharArray>::New();
  strings->SetName("strings");
  //strings->InsertNextValue("one");
  //strings->InsertNextValue("two");
  //strings->InsertNextValue("three");
  strings->InsertNextTupleValue("one");
  strings->InsertNextTupleValue("two");
  strings->InsertNextTupleValue("three");
  */
  
  //points->GetPointData()->AddArray(strings);
  //points->GetPointData()->SetScalars("strings");
  
  points->GetPointData()->SetScalars(strings);
  //points->GetPointData()->SetVectors(strings);
  
  vtkSmartPointer<vtkLabeledDataMapper> labelMapper = 
      vtkSmartPointer<vtkLabeledDataMapper>::New();
  labelMapper->SetInput(points);
  //labelMapper->SetLabelModeToLabelScalars();
  labelMapper->SetLabelModeToLabelVectors();
  vtkSmartPointer<vtkActor2D> labelActor = 
      vtkSmartPointer<vtkActor2D>::New();
  labelActor->SetMapper(labelMapper);

  
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
  renderer->AddActor(pointActor);
  renderer->AddActor(labelActor);
  
  renderer->SetBackground(1,1,1); // Background color white

  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}