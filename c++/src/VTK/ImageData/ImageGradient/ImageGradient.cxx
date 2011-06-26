#include <vtkSmartPointer.h>
#include <vtkImageMathematics.h>
#include <vtkPointData.h>
#include <vtkImageShiftScale.h>
#include <vtkImageData.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageGradient.h>
#include <vtkJPEGWriter.h>
#include <vtkImageExtractComponents.h>

int main(int argc, char *argv[])
{
  //create an image
  vtkSmartPointer<vtkImageCanvasSource2D> imageSource = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  imageSource->SetExtent(0, 200, 0, 200, 0, 0);
  imageSource->SetDrawColor(0, 0, 0);
  imageSource->FillBox(0, 200, 0, 200);
  imageSource->SetDrawColor(255, 0, 0);
  imageSource->FillBox(10, 30,  10, 30);
  imageSource->Update();
  
  vtkSmartPointer<vtkImageGradient> gradientFilter = 
      vtkSmartPointer<vtkImageGradient>::New();
  gradientFilter->SetInputConnection(imageSource->GetOutputPort());
  gradientFilter->Update();
  
  //extract the x component of the gradient
  vtkSmartPointer<vtkImageExtractComponents> extractXFilter = 
      vtkSmartPointer<vtkImageExtractComponents>::New();
  extractXFilter->SetComponents(0);
  extractXFilter->SetInputConnection(gradientFilter->GetOutputPort());
  extractXFilter->Update();
  
  //write a file of the magnitude of the x gradient
  double xRange[2];
  extractXFilter->GetOutput()->GetPointData()->GetScalars()->GetRange( xRange );
  cout << "xRange: " << xRange[0] << " " << xRange[1] << endl;
  
  {
  vtkSmartPointer<vtkImageMathematics> imageAbs = 
  vtkSmartPointer<vtkImageMathematics>::New();
  imageAbs->SetOperationToAbsoluteValue();
  imageAbs->SetInputConnection(extractXFilter->GetOutputPort());
  imageAbs->Update();
  
  vtkSmartPointer<vtkImageShiftScale> shiftScale = 
      vtkSmartPointer<vtkImageShiftScale>::New();
  shiftScale->SetOutputScalarTypeToUnsignedChar();
  shiftScale->SetScale( 255 / xRange[1] );
  shiftScale->SetInputConnection(imageAbs->GetOutputPort());
  shiftScale->Update();
      
  vtkSmartPointer<vtkJPEGWriter> xWriter = 
      vtkSmartPointer<vtkJPEGWriter>::New();
  xWriter->SetInputConnection(shiftScale->GetOutputPort());
  
  xWriter->SetFileName("x.jpg");
  xWriter->Write();
  }
  
  //extract the y component of the gradient
  vtkSmartPointer<vtkImageExtractComponents> extractYFilter = 
      vtkSmartPointer<vtkImageExtractComponents>::New();
  extractYFilter->SetComponents(1);
  extractYFilter->SetInputConnection(gradientFilter->GetOutputPort());
  extractYFilter->Update();
  
  double yRange[2];
  extractYFilter->GetOutput()->GetPointData()->GetScalars()->GetRange( yRange );
  
  //write a file of the magnitude of the y gradient
  {
  vtkSmartPointer<vtkImageMathematics> imageAbs = 
    vtkSmartPointer<vtkImageMathematics>::New();
  imageAbs->SetOperationToAbsoluteValue();
  imageAbs->SetInputConnection(extractYFilter->GetOutputPort());
  imageAbs->Update();
  
  vtkSmartPointer<vtkImageShiftScale> shiftScale = 
      vtkSmartPointer<vtkImageShiftScale>::New();
  shiftScale->SetOutputScalarTypeToUnsignedChar();
  shiftScale->SetScale( 255 / yRange[1] );
  shiftScale->SetInputConnection(imageAbs->GetOutputPort());
  shiftScale->Update();
  
  vtkSmartPointer<vtkJPEGWriter> yWriter = 
      vtkSmartPointer<vtkJPEGWriter>::New();
  yWriter->SetInputConnection(shiftScale->GetOutputPort());
  yWriter->SetFileName("y.jpg");
  yWriter->Write();
  }
  
  return EXIT_SUCCESS;
}
