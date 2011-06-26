#include <vtkSmartPointer.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkCellArray.h>
#include <vtkIdList.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>

int main(int argc, char *argv[])
{
  //parse command line arguments
  if(argc != 2)
    {
    cout << "Required arguments: OutputFilename" << endl;
    return EXIT_FAILURE;
    }
  
  vtkstd::string outputFilename = argv[1];
   
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vertices = 
      vtkSmartPointer<vtkCellArray>::New();

  vtkIdType pid[1];
  pid[0] = points->InsertNextPoint(1, 0, 0);
  vertices->InsertNextCell(1,pid);
  pid[0] = points->InsertNextPoint(0, 1, 0);
  vertices->InsertNextCell(1,pid);
  pid[0] = points->InsertNextPoint(0, 0, 1);
  vertices->InsertNextCell(1,pid);
  
  vtkSmartPointer<vtkUnsignedCharArray> colors = 
      vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");

  for ( unsigned int i = 0; i < points->GetNumberOfPoints(); i++ )
    {
    unsigned char colorArray[3] = {255, 0, 0}; //red
    colors->InsertNextTupleValue(colorArray);
    }

  /*
  vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
  if(HasTriangles)
  {
    for(unsigned int i = 0; i < Model.VertexList.size(); i++)
    {
      vector<int> vlist = Model.VertexList[i];
      vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
      triangle->GetPointIds()->SetId(0,vlist[0]);
      triangle->GetPointIds()->SetId(1,vlist[1]);
      triangle->GetPointIds()->SetId(2,vlist[2]);
      triangles->InsertNextCell(triangle);
    }
  }

  */
  vtkSmartPointer<vtkUnstructuredGrid> ug = 
      vtkSmartPointer<vtkUnstructuredGrid>::New();
  ug->SetPoints(points);
	
	//add triangles
  /*
  if(HasTriangles)
    {
    //ug->SetCells(VTK_TRIANGLE, triangles);
    }
  else
    {
    ug->SetCells(VTK_VERTEX, vertices);
    }
  */
    //add colors
  ug->GetPointData()->AddArray(colors);
  
  vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer = 
      vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
  writer->SetFileName(outputFilename.c_str());
  writer->SetInput(ug);
  writer->Write();	

  return EXIT_SUCCESS;
}

