#include <vtkImageBlend.h>
#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkPNGWriter.h>
#include <vtkSmartPointer.h>
#include <vtkFreeTypeUtilities.h>
#include <vtkTextProperty.h>
#include <vtkImageCanvasSource2D.h>

// must build VTK with VTK_USE_SYSTEM_FREETYPE=ON

int main(int argc, char* argv[])
{
  // Create a black image with a red circle of radius 5 centered at (9,10)
  vtkSmartPointer<vtkImageCanvasSource2D> drawing = 
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  drawing->SetScalarTypeToUnsignedChar();
  drawing->SetNumberOfScalarComponents(3);
  drawing->SetExtent(0, 100, 0, 200, 0, 0);
  drawing->FillBox(0,20,0,50);
  drawing->SetDrawColor(255, 0, 0, 0);
  drawing->DrawCircle(9, 10, 5);
  drawing->Update();

  // Create an image of text
  vtkSmartPointer<vtkFreeTypeUtilities> freeType = vtkSmartPointer<vtkFreeTypeUtilities>::New();
  vtkSmartPointer<vtkTextProperty> textProperty = vtkSmartPointer<vtkTextProperty>::New();
  textProperty->SetColor( 0.0,0.0,1.0 ); // blue
  textProperty->SetFontSize(10);
  
  vtkSmartPointer<vtkImageData> textImage = vtkSmartPointer<vtkImageData>::New();
  freeType->RenderString(textProperty, "Test String", 20, 40, textImage);

  // Combine the images
  vtkSmartPointer<vtkImageBlend> blend =
    vtkSmartPointer<vtkImageBlend>::New();
  blend->AddInputConnection(drawing->GetOutputPort());
  blend->AddInputConnection(textImage->GetProducerPort());
  blend->SetOpacity(0,.5);
  blend->SetOpacity(1,.5);
  blend->Update();  
   
  vtkSmartPointer<vtkPNGWriter> writer =
    vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName("output.png");
  writer->SetInputConnection(blend->GetOutputPort());
  writer->Write();
  
  return EXIT_SUCCESS;
}
