#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkObjectFactory.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkCellLocator.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static KeyPressInteractorStyle* New();
 
    KeyPressInteractorStyle() : CellsPerBucket(1) {}
    
    virtual void OnChar() 
    {
      vtkRenderWindowInteractor *rwi = this->Interactor;    
      char ch = rwi->GetKeyCode() ;
 
      switch (ch) 
      {
        case 'n':
          cout << "Next." << endl;
          this->CellsPerBucket++;
          break;
        case 'p':
          cout << "Previous." << endl;
          if(this->CellsPerBucket > 1)
          {
            this->CellsPerBucket--;
          }
          break ;
        default:
          cout << "An unhandled key was pressed." << endl;
          break;
      }
 
      cout << "CellsPerBucket = " << this->CellsPerBucket << endl;
      
      //Create the tree
      cellLocator->SetNumberOfCellsPerBucket(this->CellsPerBucket);
      cellLocator->BuildLocator();
      cellLocator->GenerateRepresentation(1, polydata);
      cout << "There are " << polydata->GetNumberOfCells() << " cells." << endl;
      
      renderWindow->Render();
      
      // forward events
      vtkInteractorStyleTrackballCamera::OnChar();
    }

    vtkCellLocator* cellLocator;
    vtkRenderWindow* renderWindow;
    vtkPolyData* polydata;
    
  private:
    unsigned int CellsPerBucket;
};


vtkStandardNewMacro(KeyPressInteractorStyle);


int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetThetaResolution(20);
  sphereSource->SetPhiResolution(20);
  sphereSource->Update();
  
  //Create the tree
  vtkSmartPointer<vtkCellLocator> cellLocator = 
      vtkSmartPointer<vtkCellLocator>::New();
  cellLocator->SetDataSet(sphereSource->GetOutput());
  cellLocator->AutomaticOn();
  cellLocator->SetNumberOfCellsPerBucket(2);
  cellLocator->BuildLocator();
  
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  cellLocator->GenerateRepresentation(cellLocator->GetLevel(), polydata);
  
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(polydata);

  // create an actor
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = 
      vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = 
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // an interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = 
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkSmartPointer<KeyPressInteractorStyle> style = vtkSmartPointer<KeyPressInteractorStyle>::New();
  style->cellLocator = cellLocator;
  style->renderWindow = renderWindow;
  style->polydata = polydata;
  
  renderWindowInteractor->SetInteractorStyle( style );
  
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white

  renderWindow->Render();
  
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;

}
