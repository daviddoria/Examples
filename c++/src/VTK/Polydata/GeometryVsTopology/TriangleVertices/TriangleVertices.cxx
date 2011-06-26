#include "vtkCellArray.h"
#include "vtkSmartPointer.h"
#include "vtkPoints.h"
#include "vtkXMLPolyDataWriter.h"
#include "vtkPolyData.h"

int main(int argc, char **argv)
{
  //Setup point coordinates
  double X[3] = {1.0, 0.0, 0.0};
  double Y[3] = {0.0, 0.0, 1.0};
  double Z[3] = {0.0, 0.0, 0.0};

  //Create points and add a vertex at each point. Really what you are doing is adding
  //cells to the polydata, and the cells only contain 1 element, so they are, by definition,
  //0-D topology (vertices).
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> Vertices = vtkSmartPointer<vtkCellArray>::New();

  for ( unsigned int i = 0; i < 3; ++i )
  {
    //Declare a variable to store the index of the point that gets added. This behaves just like an unsigned int.
    vtkIdType pid[1];
    
    //Add a point to the polydata and save its index, which we will use to create the vertex on that point.
    pid[0] = Points->InsertNextPoint ( X[i], Y[i], Z[i] );
    
    //create a vertex cell on the point that was just added.
    Vertices->InsertNextCell ( 1,pid );
  }

  //create a polydata object
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  //set the points and vertices we created as the geometry and topology of the polydata
  polydata->SetPoints ( Points );
  polydata->SetVerts ( Vertices );

  //write the polydata to a file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName ( "TriangleVerts.vtp" );
  writer->SetInput ( polydata );
  writer->Write();


}
