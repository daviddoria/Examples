#include <vtkSmartPointer.h>
#include <vtkAssemblyPath.h>
#include <vtkAssemblyNode.h>
#include <vtkProperty2D.h>
#include <vtkImageActor.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkJPEGReader.h>
#include <vtkBorderWidget.h>
#include <vtkBorderRepresentation.h>
#include <vtkCommand.h>
#include <vtkPropPicker.h>
#include <vtkInteractorStyleImage.h>
#include <vtkCoordinate.h>

class vtkBorderCallback : public vtkCommand
{
  public:
    vtkBorderCallback(){}
    
    static vtkBorderCallback *New()
    {
      return new vtkBorderCallback;
    }
    
    virtual void Execute(vtkObject *caller, unsigned long, void*)
    {
      
      vtkBorderWidget *borderWidget = 
          reinterpret_cast<vtkBorderWidget*>(caller);
      
      //get the display coordinates of the two corners of the box
      vtkCoordinate* lowerLeftCoordinate = static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetPositionCoordinate();
      int* lowerLeft = new int[3];
      lowerLeft = lowerLeftCoordinate ->GetComputedDisplayValue(this->Renderer);
      cout << "Lower left coordinate: " << lowerLeft[0] << ", " << lowerLeft[1] << endl;
      lowerLeft[2] = 0;
      
      vtkCoordinate* upperRightCoordinate = static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetPosition2Coordinate();
      int* upperRight = new int[3];
      upperRight = upperRightCoordinate ->GetComputedDisplayValue(this->Renderer);
      cout << "Upper right coordinate: " << upperRight[0] << ", " << upperRight[1] << endl;
      upperRight[2] = 0;
      
      //pick at the two corners of the box
      vtkSmartPointer<vtkPropPicker> picker = 
          vtkSmartPointer<vtkPropPicker>::New();
            
      int valid1 = picker->Pick( lowerLeft[0],
                                 lowerLeft[1], 0.0, this->Renderer );
      
      vtkAssemblyPath* path = picker->GetPath();
      
      vtkProp* pickedProp = NULL;

      bool validPick = false;
      if( path )
      {
        cout << "There are " << path->GetNumberOfItems() << " items in the path." << endl;
        vtkCollectionSimpleIterator sit;
        path->InitTraversal( sit );
        vtkAssemblyNode *node;
        for( int i = 0; i < path->GetNumberOfItems(); ++i )
        {
          node = path->GetNextNode( sit );
          pickedProp = node->GetViewProp();
          if( this->ImageActor == vtkImageActor::SafeDownCast( pickedProp ) )
          {
            cout << "Correct actor picked." << endl;
            validPick = true;
            break;
          }
        }
      }

      if(!validPick)
      {
        cout << "Off Image" << endl;
      }
      else
      { 
        double pos1[3];
        picker->GetPickPosition( pos1 );
        cout << "Lower Left Pick: " << pos1[0] << " " << pos1[1] << endl;
      }    
      /*
      if(valid1 != 0)
      {
      cout << "Valid1: " << valid1 << endl;
      double pos1[3];
      picker->GetPickPosition( pos1 );
      cout << "Lower Left Pick: " << pos1[0] << " " << pos1[1] << endl;
    }
      else
      {
      cout << "Left - nothing was picked" << endl;
    }  
      */
      
      /*
      int valid2 = picker->Pick( upperRight[0],
      upperRight[1], 0.0, this->Renderer );
      if(valid2 != 0)
      {
      cout << "Valid2: " << valid2 << endl;
      double pos2[3];
      picker->GetPickPosition( pos2 );
      cout << "Upper right Pick: " << pos2[0] << " " << pos2[1] << endl;
    }
      else
      {
      cout << "Right - nothing was picked" << endl;
    }
      */
      cout << endl;
    }
    
    void SetRenderer(vtkSmartPointer<vtkRenderer> ren) {this->Renderer = ren;}
    void SetImageActor(vtkSmartPointer<vtkImageActor> im) {this->ImageActor = im;}
    
  private:
    vtkSmartPointer<vtkRenderer> Renderer;
    vtkSmartPointer<vtkImageActor> ImageActor;
    
};

int main (int argc, char *argv[])
{
  //parse input arguments
  if ( argc != 2 )
  {
    cout << "Required parameters: Filename" << endl;
    return EXIT_FAILURE;
  }
 
  std::string InputFilename = argv[1];
 
  //read the image
  vtkSmartPointer<vtkJPEGReader> jPEGReader = 
      vtkSmartPointer<vtkJPEGReader>::New();
  jPEGReader->SetFileName ( InputFilename.c_str() );
  jPEGReader->Update();

  vtkSmartPointer<vtkImageActor> actor = 
      vtkSmartPointer<vtkImageActor>::New();
  actor->SetInput(jPEGReader->GetOutput());
  
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
  
  vtkSmartPointer<vtkInteractorStyleImage> style = 
      vtkSmartPointer<vtkInteractorStyleImage>::New();
 
  renderWindowInteractor->SetInteractorStyle( style );
  
  vtkSmartPointer<vtkBorderWidget> borderWidget = 
      vtkSmartPointer<vtkBorderWidget>::New();
  borderWidget->SetInteractor(renderWindowInteractor);
  static_cast<vtkBorderRepresentation*>(borderWidget->GetRepresentation())->GetBorderProperty()->SetColor(1,0,0);
  borderWidget->SelectableOff();
  
  vtkSmartPointer<vtkBorderCallback> borderCallback = 
      vtkSmartPointer<vtkBorderCallback>::New();
  borderCallback->SetRenderer(renderer);
  borderCallback->SetImageActor(actor);
  
  borderWidget->AddObserver(vtkCommand::InteractionEvent,borderCallback);
  
  // add the actors to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(1,1,1);
  
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindow->Render();
  borderWidget->On();
  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
