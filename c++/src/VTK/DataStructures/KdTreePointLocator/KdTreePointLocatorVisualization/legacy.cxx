#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkKdTreePointLocator.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPointSource.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkAppendPolyData.h>

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
    vtkSmartPointer<vtkPointSource> pointSource =
      vtkSmartPointer<vtkPointSource>::New();
    pointSource->SetNumberOfPoints(1000);
    pointSource->Update();
    input = pointSource->GetOutput();
    }
  
  std::cout << "Number of points in input: " << input->GetNumberOfPoints() << std::endl;
  
  //Create the tree
  vtkSmartPointer<vtkKdTreePointLocator> kdTree =
    vtkSmartPointer<vtkKdTreePointLocator>::New();
  kdTree->SetDataSet(input);
  kdTree->AutomaticOff();
  kdTree->SetMaxLevel(6);
  kdTree->BuildLocator();
  std::cout << "Generating " << kdTree->GetLevel() << " levels." << std::endl;
  
  for(unsigned int i = 0; i <= kdTree->GetLevel(); i++)
    {
    
    vtkSmartPointer<vtkPolyData> polydata = 
      vtkSmartPointer<vtkPolyData>::New();
    kdTree->GenerateRepresentation(i, polydata);

    vtkSmartPointer<vtkAppendPolyData> appendFilter =
      vtkSmartPointer<vtkAppendPolyData>::New();
    appendFilter->AddInput(polydata);
    appendFilter->AddInput(input);
    appendFilter->Update();
  
    std::cout << "Level " << i << std::endl;
    RenderPolyData(appendFilter->GetOutput());
  
    }
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
  actor->GetProperty()->SetRepresentationToWireframe();
  
  //Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
}
