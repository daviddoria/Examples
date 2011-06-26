#include "ui_SimpleView.h"
#include "SimpleView.h"

#include <vtkSmartPointer.h>
#include <vtkBoxWidget2.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>

#include "InteractorStyle.h"

// Constructor
SimpleView::SimpleView()
{
  this->ui = new Ui_SimpleView;
  this->ui->setupUi(this);

  // VTK Renderer
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();

  // Connect VTK with Qt
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
  renderer->ResetCamera();

  vtkSmartPointer<InteractorStyle> style =
    vtkSmartPointer<InteractorStyle>::New();
  style->SetInteractor(this->ui->qvtkWidget->GetInteractor());
  style->Initialize();
  
  this->ui->qvtkWidget->GetInteractor()->SetInteractorStyle(style);
};
