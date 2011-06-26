#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkJPEGWriter.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageMask.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderer.h>
#include <vtkJPEGReader.h>
#include <vtkImageActor.h>
#include <vtkImageEllipsoidSource.h>
#include <vtkImageCast.h>

int main(int, char *[])
{
  // Create an image of a rectangle
  vtkSmartPointer<vtkImageCanvasSource2D> source = 
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  source->SetScalarTypeToUnsignedChar();
  source->SetNumberOfScalarComponents(3);
  source->SetExtent(0, 200, 0, 200, 0, 0);
  
  // Create a red image
  source->SetDrawColor(255,0,0);
  source->FillBox(0,200,0,200);
  
  source->Update();
  
  // Create a rectanglular mask
  vtkSmartPointer<vtkImageCanvasSource2D> maskSource = 
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  maskSource->SetScalarTypeToUnsignedChar();
  maskSource->SetNumberOfScalarComponents(1);
  maskSource->SetExtent(0, 200, 0, 200, 0, 0);
  
  // Initialize the mask to black
  maskSource->SetDrawColor(0,0,0);
  maskSource->FillBox(0,200,0,200);
  
  // Create a square
  maskSource->SetDrawColor(255,255,255); //anything non-zero means "make the output pixel equal the input pixel." If the mask is zero, the output pixel is set to MaskedValue
  maskSource->FillBox(100,120,100,120);
  maskSource->Update();

  vtkSmartPointer<vtkImageMask> maskFilter = 
    vtkSmartPointer<vtkImageMask>::New();
  maskFilter->SetImageInput(source->GetOutput());
  maskFilter->SetMaskInput(maskSource->GetOutput());
  maskFilter->SetMaskedOutputValue(0,1,0);
  maskFilter->Update();

  vtkSmartPointer<vtkImageMask> inverseMaskFilter =
    vtkSmartPointer<vtkImageMask>::New();
  inverseMaskFilter->SetImageInput(source->GetOutput());
  inverseMaskFilter->SetMaskInput(maskSource->GetOutput());
  inverseMaskFilter->SetMaskedOutputValue(0,1,0);
  inverseMaskFilter->NotMaskOn();
  inverseMaskFilter->Update();

  // Create actors
  vtkSmartPointer<vtkImageActor> originalActor =
    vtkSmartPointer<vtkImageActor>::New();
  originalActor->SetInput(source->GetOutput());

  vtkSmartPointer<vtkImageActor> maskActor =
    vtkSmartPointer<vtkImageActor>::New();
  maskActor->SetInput(maskSource->GetOutput());

  vtkSmartPointer<vtkImageActor> maskedActor =
    vtkSmartPointer<vtkImageActor>::New();
  maskedActor->SetInput(maskFilter->GetOutput());

  vtkSmartPointer<vtkImageActor> inverseMaskedActor =
    vtkSmartPointer<vtkImageActor>::New();
  inverseMaskedActor->SetInput(inverseMaskFilter->GetOutput());

  // Define viewport ranges
  // (xmin, ymin, xmax, ymax)
  double originalViewport[4] = {0.0, 0.0, 0.25, 1.0};
  double maskViewport[4] = {0.25, 0.0, 0.5, 1.0};
  double maskedViewport[4] = {0.5, 0.0, 0.75, 1.0};
  double inverseMaskedViewport[4] = {0.75, 0.0, 1.0, 1.0};

  // Setup renderers
  vtkSmartPointer<vtkRenderer> originalRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  originalRenderer->SetViewport(originalViewport);
  originalRenderer->AddActor(originalActor);
  originalRenderer->ResetCamera();
  originalRenderer->SetBackground(.4, .5, .6);

  vtkSmartPointer<vtkRenderer> maskRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  maskRenderer->SetViewport(maskViewport);
  maskRenderer->AddActor(maskActor);
  maskRenderer->ResetCamera();
  maskRenderer->SetBackground(.4, .5, .6);

  vtkSmartPointer<vtkRenderer> maskedRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  maskedRenderer->SetViewport(maskedViewport);
  maskedRenderer->AddActor(maskedActor);
  maskedRenderer->ResetCamera();
  maskedRenderer->SetBackground(.4, .5, .6);

  vtkSmartPointer<vtkRenderer> inverseMaskedRenderer =
    vtkSmartPointer<vtkRenderer>::New();
  inverseMaskedRenderer->SetViewport(inverseMaskedViewport);
  inverseMaskedRenderer->AddActor(inverseMaskedActor);
  inverseMaskedRenderer->ResetCamera();
  inverseMaskedRenderer->SetBackground(.4, .5, .7);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
    vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(1200, 300);
  renderWindow->AddRenderer(originalRenderer);
  renderWindow->AddRenderer(maskRenderer);
  renderWindow->AddRenderer(maskedRenderer);
  renderWindow->AddRenderer(inverseMaskedRenderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  vtkSmartPointer<vtkInteractorStyleImage> style =
    vtkSmartPointer<vtkInteractorStyleImage>::New();

  renderWindowInteractor->SetInteractorStyle(style);

  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->Initialize();

  renderWindowInteractor->Start();
  
  return EXIT_SUCCESS;
}
