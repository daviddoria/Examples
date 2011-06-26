#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkImageViewer2.h>
#include <vtkJPEGReader.h>
#include <vtkImageCanvasSource2D.h>

int main(int argc, char* argv[])
{
  vtkSmartPointer<vtkImageCanvasSource2D> drawing = 
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  drawing->SetScalarTypeToUnsignedChar();
  drawing->SetNumberOfScalarComponents(3);
  drawing->SetExtent(0, 20, 0, 50, 0, 0);
  drawing->FillBox(0,20,0,50);
  drawing->SetDrawColor(255, 0, 0, 0);
  drawing->DrawCircle(9, 10, 5);
  drawing->Update();
  
  vtkSmartPointer<vtkImageViewer2> imageViewer = 
      vtkSmartPointer<vtkImageViewer2>::New();
  imageViewer->SetInput(drawing->GetOutput());
  imageViewer->Render();
  imageViewer->GetRenderer()->ResetCamera();
  
  imageViewer->Render();
  
  return EXIT_SUCCESS;
}

