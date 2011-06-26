#include "ui_SimpleView.h"
#include "SimpleView.h"

#include <vtkSmartPointer.h>
#include <vtkBoxWidget2.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>

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

  this->Style = vtkSmartPointer<InteractorStyle>::New();
  //this->Style->SetCurrentRenderer(this->ui->qvtkWidget->GetRenderWindow()
    //                ->GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
  this->Style->SetCurrentRenderer(renderer);
  this->Style->SetInteractor(this->ui->qvtkWidget->GetInteractor());
  this->Style->Initialize();
  
  this->ui->qvtkWidget->GetInteractor()->SetInteractorStyle(this->Style);
};
