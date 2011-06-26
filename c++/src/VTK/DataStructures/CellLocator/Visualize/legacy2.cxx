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
  
    KeyPressInteractorStyle() : CellsPerBucket(10) {}
  
    virtual void OnChar() 
    {
      vtkRenderWindowInteractor *rwi = this->Interactor;    
      char ch = rwi->GetKeyCode() ;
      
      switch (ch) 
        {
        case 'n':
          if(this->CellsPerBucket == this->CellLocator->GetDataSet()->GetNumberOfCells())
            {
            cout << "CellsPerBucket is set to the number of cells in the sphere. "  
                << " Increasing CellsPerBucket will not make a difference." << endl;
            }
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
      this->CellLocator->SetNumberOfCellsPerBucket(this->CellsPerBucket);
      this->CellLocator->BuildLocator();
      this->CellLocator->GenerateRepresentation(
          this->CellLocator->GetLevel(), this->PolyData);
      cout << "There are " << this->CellLocator->GetLevel() 
          << " levels in the locator." << endl;
      cout << "There are " << this->PolyData->GetNumberOfCells() 
          << " cells in the representation of the locator at this level." << endl;
      
      RenderWindow->Render();
      
      // forward events
      vtkInteractorStyleTrackballCamera::OnChar();
    }
 
    vtkCellLocator* CellLocator;
    vtkRenderWindow* RenderWindow;
    vtkPolyData* PolyData;
  
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
  cellLocator->SetNumberOfCellsPerBucket(10);
  cellLocator->BuildLocator();
  
  vtkSmartPointer<vtkPolyData> polyData = 
      vtkSmartPointer<vtkPolyData>::New();
  cellLocator->GenerateRepresentation(cellLocator->GetLevel(), polyData);
  
  vtkSmartPointer<vtkPolyDataMapper> mapper = 
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(polyData);
  
  // create an actor
  vtkSmartPointer<vtkActor> actor = 
      vtkSmartPointer<vtkActor>::New();
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
  
  vtkSmartPointer<KeyPressInteractorStyle> style = 
      vtkSmartPointer<KeyPressInteractorStyle>::New();
  style->CellLocator = cellLocator;
  style->RenderWindow = renderWindow;
  style->PolyData = polyData;
  
  renderWindowInteractor->SetInteractorStyle( style );
  
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1); // Background color white
 
  renderWindow->Render();
  
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;

}

