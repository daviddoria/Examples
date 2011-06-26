#include "QuickView.h"

#include "itkImage.h"

#include "vtkImageViewer.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkImageActor.h"
#include "vtkInteractorStyleImage.h"
#include "vtkRenderer.h"

#include <itkImageToVTKImageFilter.h>

void QuickView::AddImage(itk::Image<unsigned char, 2>::Pointer  image)
{
  this->Images.push_back(image);
}

/*
void QuickView::Visualize()
{
  // Setup the render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  unsigned int rendererSize = 300;
  renderWindow->SetSize(rendererSize * this->Images.size(), rendererSize);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);


  typedef itk::ImageToVTKImageFilter<itk::Image<unsigned char, 2> > ConnectorType;
  ConnectorType::Pointer connector = ConnectorType::New();
  connector->SetInput(this->Images[0]);
  connector->Update();

  // (xmin, ymin, xmax, ymax)
  //double viewport[4] = {static_cast<double>(i)*step, 0.0, static_cast<double>(i+1)*step, 1.0};
  //viewports.push_back(viewport);
  vtkSmartPointer<vtkImageActor> actor =
    vtkSmartPointer<vtkImageActor>::New();
  actor->SetInput(connector->GetOutput());

  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(renderer);

  renderer->AddActor(actor);
  renderer->ResetCamera();

  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  interactor->SetInteractorStyle(style);
  interactor->Start();
}
*/


void QuickView::Visualize()
{
  // Setup the render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  unsigned int rendererSize = 300;
  renderWindow->SetSize(rendererSize * this->Images.size(), rendererSize);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  // Render all of the images
  double step = 1./(static_cast<double>(this->Images.size()));
  std::vector<double*> viewports;

  typedef itk::ImageToVTKImageFilter<itk::Image<unsigned char, 2> > ConnectorType;
  std::vector<ConnectorType::Pointer> connectors; // Force the connectors to persist (not lose scope) after each iteration of the loop

  for(unsigned int i = 0; i < this->Images.size(); i++)
    {

    ConnectorType::Pointer connector = ConnectorType::New();
    connectors.push_back(connector);
    connector->SetInput(this->Images[i]);
    connector->Update();

    // (xmin, ymin, xmax, ymax)
    double viewport[4] = {static_cast<double>(i)*step, 0.0, static_cast<double>(i+1)*step, 1.0};
    viewports.push_back(viewport);
    vtkSmartPointer<vtkImageActor> actor =
      vtkSmartPointer<vtkImageActor>::New();
    actor->SetInput(connector->GetOutput());

    // Setup both renderers
    vtkSmartPointer<vtkRenderer> renderer =
      vtkSmartPointer<vtkRenderer>::New();
    renderWindow->AddRenderer(renderer);
    renderer->SetViewport(viewports[i]);
    //renderer->SetBackground(.6, .5, .4);

    renderer->AddActor(actor);
    renderer->ResetCamera();
    }

  renderWindow->Render();

  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();
  interactor->SetInteractorStyle(style);
  interactor->Start();
}
