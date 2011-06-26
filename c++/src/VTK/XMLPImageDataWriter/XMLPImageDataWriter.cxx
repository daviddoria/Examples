#include <vtkSmartPointer.h>
#include <vtkXMLPImageDataWriter.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageDataStreamer.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkImageCanvasSource2D> drawing =
    vtkSmartPointer<vtkImageCanvasSource2D>::New();
  drawing->SetNumberOfScalarComponents(3);
  drawing->SetScalarTypeToUnsignedChar();
  drawing->SetExtent(0, 20, 0, 50, 0, 1);
  drawing->SetDrawColor(255.0, 255.0, 255.0);
  drawing->DrawCircle(5, 5, 3);

  int numberOfPieces = 4;
  
  //vtkSmartPointer<vtkImageDataStreamer> streamer = 
    //vtkSmartPointer<vtkImageDataStreamer>::New();
  //streamer->SetInputConnection(drawing->GetOutputPort());

  vtkSmartPointer<vtkXMLPImageDataWriter> writer =
    vtkSmartPointer<vtkXMLPImageDataWriter>::New();
  //writer->SetInputConnection(streamer->GetOutputPort());
  writer->SetInputConnection(drawing->GetOutputPort());
  writer->SetFileName("Test.pvti");
  writer->SetNumberOfPieces(numberOfPieces);
  writer->SetEndPiece(numberOfPieces-1);
  writer->SetStartPiece(0);
  writer->Update();

  return EXIT_SUCCESS;
}
