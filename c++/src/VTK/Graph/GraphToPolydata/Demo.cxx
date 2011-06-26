#include <vtkSmartPointer.h>
#include <vtkGraphReader.h>
#include <vtkGraphToPolyData.h>
#include <vtkXMLPolyDataWriter.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkGraphReader> reader = 
    vtkSmartPointer<vtkGraphReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();
  
  //convert the graph to a polydata
  vtkSmartPointer<vtkGraphToPolyData> graphToPolyData = 
      vtkSmartPointer<vtkGraphToPolyData>::New();
  graphToPolyData->SetInputConnection(reader->GetOutputPort());
  graphToPolyData->Update();
  
  //write the result to a vtp file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(graphToPolyData->GetOutput());
  writer->SetFileName(argv[2]);
  writer->Write();
  
  return EXIT_SUCCESS;
}
