#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageViewer2.h>
#include <vtkJPEGReader.h>
#include <vtkImageCanvasSource2D.h>
//#include <iostream>
//#include <string>

int main(int argc, char* argv[])
{
  /*
  if(argc != 2)
    { 
    cout << "Required parameters: Filename" << endl;
    return EXIT_FAILURE;
    }
  
  std::string inputFilename = argv[1];
  
  vtkSmartPointer<vtkJPEGReader> jpegReader = 
      vtkSmartPointer<vtkJPEGReader>::New();
  jpegReader->SetFileName(inputFilename.c_str());
  jpegReader->Update();
  */
  
  
  vtkSmartPointer<vtkImageCanvasSource2D> drawing = 
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  drawing->SetScalarTypeToUnsignedChar();
  drawing->SetNumberOfScalarComponents(3);
  drawing->SetExtent(0, 20, 0, 50, 0, 0);
  drawing->FillBox(0,20,0,50);
 
  //Draw a red circle of radius 5 centered at (9,10)
  //drawing->SetDrawColor(255, 0, 0, 0.5);
  drawing->SetDrawColor(255, 0, 0, 0);
  drawing->DrawCircle(9, 10, 5);
  drawing->Update();
  
  vtkSmartPointer<vtkImageViewer2> imageViewer = 
      vtkSmartPointer<vtkImageViewer2>::New();
  
  //imageViewer->SetInput(jpegReader->GetOutput());
  imageViewer->SetInput(drawing->GetOutput());
  
  imageViewer->GetRenderWindow()->SetSize(500, 500);
  
  imageViewer->Render();
  imageViewer->GetRenderer()->ResetCamera();
  
  imageViewer->Render();
  
  return EXIT_SUCCESS;
}

