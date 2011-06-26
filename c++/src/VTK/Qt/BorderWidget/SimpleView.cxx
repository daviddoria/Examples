#include "ui_SimpleView.h"
#include "SimpleView.h"

#include <vtkSmartPointer.h>
#include <vtkBorderWidget.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkSphereSource.h>

class BorderCallback : public vtkCommand
{
  public:
    BorderCallback(){}

    static BorderCallback *New()
    {
      return new BorderCallback;
    }

    virtual void Execute(vtkObject *caller, unsigned long, void*)
    {
      vtkBorderWidget *borderWidget =
          reinterpret_cast<vtkBorderWidget*>(caller);
    }

};

// Constructor
SimpleView::SimpleView()
{
  this->ui = new Ui_SimpleView;
  this->ui->setupUi(this);

  // Sphere
  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
    vtkSmartPointer<vtkPolyDataMapper>::New();
  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
  vtkSmartPointer<vtkActor> sphereActor =
    vtkSmartPointer<vtkActor>::New();
  sphereActor->SetMapper(sphereMapper);

  // VTK Renderer
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(sphereActor);

  // Connect VTK with Qt
  this->ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);

  // Add a border widget to the right renderer
  this->BorderWidget = vtkSmartPointer<vtkBorderWidget>::New();
  this->BorderWidget->SetInteractor(this->ui->qvtkWidget->GetInteractor());
  this->BorderWidget->On();
};
