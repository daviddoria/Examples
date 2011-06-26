#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkDelimitedTextReader.h>
#include <vtkTable.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>

int main(int argc, char *argv[])
{
  if(argc != 3)
    {
    std::cout << "Required parameters: InputFilename OutputFilename" << std::endl;
    return EXIT_FAILURE;
    }

  std::string inputFilename = argv[1];
  std::string outputFilename = argv[2];
  
  vtkSmartPointer<vtkDelimitedTextReader> reader = 
    vtkSmartPointer<vtkDelimitedTextReader>::New();
  reader->SetFileName(inputFilename.c_str());
  reader->DetectNumericColumnsOn();
  //char delim = ' ';
  reader->SetFieldDelimiterCharacters(" ");
  reader->Update();
  
  vtkTable* table = reader->GetOutput();
  
  vtkSmartPointer<vtkPoints> points = 
    vtkSmartPointer<vtkPoints>::New();
    
  vtkSmartPointer<vtkFloatArray> normals = 
    vtkSmartPointer<vtkFloatArray>::New();
  normals->SetNumberOfComponents(3); //3d normals (ie x,y,z)
  normals->SetName("Normals");
  
  vtkSmartPointer<vtkUnsignedCharArray> colors = 
    vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");
  
  //std::cout << "Table has " << table->GetNumberOfRows() << " rows." << std::endl;
  //std::cout << "Table has " << table->GetNumberOfColumns() << " columns." << std::endl;
  
  for(unsigned int i = 0; i < table->GetNumberOfRows(); i++)
    {
    //std::cout << "x: " << (table->GetValue(i,0)).ToDouble() << " y: " << (table->GetValue(i,1)).ToDouble() << " z: " << (table->GetValue(i,2)).ToDouble();
    
    points->InsertNextPoint((table->GetValue(i,0)).ToDouble(), (table->GetValue(i,1)).ToDouble(), (table->GetValue(i,2)).ToDouble());
    
    double n[3];
    n[0] = (table->GetValue(i,3)).ToDouble();
    n[1] = (table->GetValue(i,4)).ToDouble();
    n[2] = (table->GetValue(i,5)).ToDouble();
    normals->InsertNextTuple(n);
  
    unsigned char c[3];
    c[0] = (table->GetValue(i,6)).ToUnsignedChar();
    c[1] = (table->GetValue(i,7)).ToUnsignedChar();
    c[2] = (table->GetValue(i,8)).ToUnsignedChar();
    
    colors->InsertNextTupleValue(c);
    }
  
  //std::cout << "There are " << points->GetNumberOfPoints() << " points." << std::endl;
  
  vtkSmartPointer<vtkPolyData> polydata = 
    vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  //polydata->GetPointData()->SetNormals(normals);
  polydata->GetPointData()->AddArray(normals);
  polydata->GetPointData()->AddArray(colors);
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFilename.c_str());
  writer->SetInput(polydata);
  writer->Write();

  return EXIT_SUCCESS;
}
