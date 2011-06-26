#include "ui_SimpleView.h"
#include "SimpleView.h"

#include <vtkSmartPointer.h>
#include <vtkBoxWidget2.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>

// Constructor
SimpleView::SimpleView()
{
  this->ui = new Ui_SimpleView;
  this->ui->setupUi(this);

  // VTK Renderer
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(sphereActor);

  // Connect VTK with Qt
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
  renderer->ResetCamera();

  // Add a border widget to the renderer
  this->BoxWidget = vtkSmartPointer<vtkBoxWidget2>::New();
  this->BoxWidget->SetInteractor(this->ui->qvtkWidget->GetInteractor());
  this->BoxWidget->On();
};
