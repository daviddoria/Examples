// BROKEN - do NOT use this order of interactors, etc

#include <vtkSmartPointer.h>
#include <vtkProperty.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProbeFilter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPolyDataSilhouette.h>
#include <vtkSphereSource.h>

int main(int argc, char *argv[])
{
  vtkstd::string inputFilename = argv[1];
  /*
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->Update();
  */
  vtkSmartPointer<vtkSphereSource> reader = vtkSmartPointer<vtkSphereSource>::New();
  
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());
  
  
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  
  renderer->AddActor(actor);
  renderer->ResetCamera();
  
/*
  vtkSmartPointer<vtkPolyDataSilhouette> silhouette = vtkSmartPointer<vtkPolyDataSilhouette>::New();
  silhouette->SetInputConnection(reader->GetOutputPort());
  silhouette->SetEnableFeatureAngle(false);
  silhouette->SetBorderEdges(true);
  silhouette->SetPieceInvariant(false);
  silhouette->SetDirectionToCameraOrigin();
  silhouette->SetCamera(renderer->GetActiveCamera());

  vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper2->SetInputConnection(silhouette->GetOutputPort());
  vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
  actor2->SetMapper(mapper2);
  actor2->GetProperty()->SetColor(0.5, 0.5, 1);
  renderer->AddActor(actor2);
*/
  renderWindow->SetSize(400, 400);
  renderWindow->Render();

  vtkSmartPointer<vtkRenderWindowInteractor> iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renderWindow);
  
  //iren->Initialize();
  iren->Start();

  return 0;
}
