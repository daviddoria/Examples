#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkImageAppendComponents.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkJPEGWriter.h>

int main(int argc, char *argv[])
{
  //create an image
  vtkSmartPointer<vtkImageCanvasSource2D> source0 = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  source0->SetScalarTypeToUnsignedChar();
  source0->SetNumberOfScalarComponents(1);
  source0->SetExtent(0, 100, 0, 100, 0, 0);
  source0->SetDrawColor(0, 0, 0, 0);
  source0->FillBox(0,100,0,100);
  source0->SetDrawColor(255, 0, 0, 0);
  source0->FillBox(20,40,20,40);
  source0->Update();
  
  {
  vtkSmartPointer<vtkJPEGWriter> writer =
    vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetInputConnection(source0->GetOutputPort());
  writer->SetFileName("0.jpg");
  writer->Write();
  }
  
  //create an image
  vtkSmartPointer<vtkImageCanvasSource2D> source1 = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  source1->SetScalarTypeToUnsignedChar();
  source1->SetNumberOfScalarComponents(1);
  source1->SetExtent(0, 100, 0, 100, 0, 0);
  source1->SetDrawColor(0, 0, 0, 0);
  source1->FillBox(0,100,0,100);
  //source1->SetDrawColor(0, 255, 0, 0);
  source1->SetDrawColor(255, 0, 0, 0);
  source1->FillBox(30,50,30,50);
  source1->Update();
  
  {
  vtkSmartPointer<vtkJPEGWriter> writer =
    vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetInputConnection(source1->GetOutputPort());
  writer->SetFileName("1.jpg");
  writer->Write();
  }
  
    //create an image
  vtkSmartPointer<vtkImageCanvasSource2D> source2 = 
      vtkSmartPointer<vtkImageCanvasSource2D>::New();
  source2->SetScalarTypeToUnsignedChar();
  source2->SetNumberOfScalarComponents(1);
  source2->SetExtent(0, 100, 0, 100, 0, 0);
  source2->SetDrawColor(0, 0, 0, 0);
  source2->FillBox(0,100,0,100);
  source2->SetDrawColor(255, 0, 0, 0);
  source2->FillBox(40,60,40,60);
  source2->Update();
  
  {
  vtkSmartPointer<vtkJPEGWriter> writer =
    vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetInputConnection(source2->GetOutputPort());
  writer->SetFileName("2.jpg");
  writer->Write();
  }
  
  vtkSmartPointer<vtkImageAppendComponents> appendFilter = 
      vtkSmartPointer<vtkImageAppendComponents>::New();
  appendFilter->SetInputConnection(0, source0->GetOutputPort());
  appendFilter->AddInputConnection(0, source1->GetOutputPort());
  appendFilter->AddInputConnection(0, source2->GetOutputPort());
  appendFilter->Update();
  
  vtkSmartPointer<vtkJPEGWriter> writer =
      vtkSmartPointer<vtkJPEGWriter>::New();
  writer->SetInputConnection(appendFilter->GetOutputPort());
  writer->SetFileName("output.jpg");
  writer->Write();
  
  return EXIT_SUCCESS;
}
