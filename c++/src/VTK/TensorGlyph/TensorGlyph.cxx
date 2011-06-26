#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTensorGlyph.h>
#include <vtkDataSetReader.h>
 
int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkDataSetReader> reader = 
      vtkSmartPointer<vtkDataSetReader>::New();
  reader->SetFileName("/home/doriad/src/VTKData/Data/tensors.vtk");
  
  vtkSmartPointer<vtkSphereSource> sourceGlyph =
    vtkSmartPointer<vtkSphereSource>::New();
  sourceGlyph->SetRadius(0.5);
  sourceGlyph->SetCenter(0.5, 0.0, 0.0);
  
      
  vtkSmartPointer<vtkTensorGlyph> glyph = 
      vtkSmartPointer<vtkTensorGlyph>::New();
  glyph->SetInputConnection(reader->GetOutputPort());
  glyph->SetSource(sourceGlyph->GetOutput());
  glyph->SetScaleFactor(0.25);
  glyph->SetColorModeToEigenvalues();
  
  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(glyph->GetOutputPort());
 
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