#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>

#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkXMLPolyDataReader.h>

#include <vtkModifiedBSPTree.h>

void RenderPolyData(vtkSmartPointer<vtkPolyData> polydata);

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPolyData> input;
  if(argc > 1)
    {
    vtkSmartPointer<vtkXMLPolyDataReader> reader =
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName(argv[1]);
    reader->Update();
    input = reader->GetOutput();
    }
  else
    {
    vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->Update();
    input = sphereSource->GetOutput();
    }
  
  std::cout << "Number of points in input: " << input->GetNumberOfPoints() << std::endl;
  
  //Create the tree
  vtkSmartPointer<vtkModifiedBSPTree> tree =
      vtkSmartPointer<vtkModifiedBSPTree>::New();
  tree->SetDataSet(input);
  tree->BuildLocator();
  
  vtkSmartPointer<vtkPolyData> polydata =
      vtkSmartPointer<vtkPolyData>::New();
  tree->GenerateRepresentation(tree->GetLevel(), polydata);

  std::cout << "Number of points in tree: " << polydata->GetNumberOfPoints() << std::endl;

  RenderPolyData(polydata);
  
  return EXIT_SUCCESS;
}

void RenderPolyData(vtkSmartPointer<vtkPolyData> polydata)
{

  //Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(polydata->GetProducerPort());

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

  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
}