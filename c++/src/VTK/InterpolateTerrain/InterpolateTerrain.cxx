#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkTriangle.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkLine.h>
#include <vtkImageData.h>
#include <vtkProbeFilter.h>
#include <vtkDelaunay2D.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkDoubleArray.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkImageData> image = 
      vtkSmartPointer<vtkImageData>::New();
  image->SetExtent(0, 9, 0, 9, 0, 0);
  image->SetNumberOfScalarComponents(1);
  
  /*
    for (int y = 0; y < dims[1]; y++)
      {
      for (int x = 0; x < dims[0]; x++)
        {
        image->SetScalarComponentFromDouble(x,y,0,0,2.0);
        }
      }
  */
  //Create a random set of heighs on a grid. This is often called a "terrain map"
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

	unsigned int GridSize = 10;
	for ( unsigned int x = 0; x < GridSize; x++ )
	  {
		for ( unsigned int y = 0; y < GridSize; y++ )
		  {
      double val = drand48();
			points->InsertNextPoint ( x, y, val);
      image->SetScalarComponentFromDouble(x,y,0,0,val);
		  }
	  }

	//add the grid points to a polydata object
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints ( points );

	//triangulate the grid points
	vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
	delaunay->SetInput ( polydata );
	delaunay->Update();

	vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
	writer->SetFileName ( "surface.vtp" );
	writer->SetInput ( delaunay->GetOutput() );
	writer->Write();

  vtkPolyData* surface = delaunay->GetOutput();
  
  vtkSmartPointer<vtkPoints> probePoints = 
      vtkSmartPointer<vtkPoints>::New();
  probePoints->InsertNextPoint(5.5, 5.5, 0);
  vtkSmartPointer<vtkPolyData> probePolyData = 
      vtkSmartPointer<vtkPolyData>::New();
  probePolyData->SetPoints(probePoints);
      
  vtkSmartPointer<vtkProbeFilter> probe = 
      vtkSmartPointer<vtkProbeFilter>::New();
  probe->SetInput(image);
  probe->SetSource(probePolyData);
  probe->Update();
  
  vtkDataSet* output = probe->GetOutput(); 

  vtkDataArray* data = probe->GetOutput()->GetPointData()->GetScalars();
  if(data)
    {
    cout << "data is valid" << endl;
    }
  else
    {
    cout << "data is NOT valid" << endl;
    }
  
  //vtkDoubleArray* doubleData = dynamic_cast<vtkDoubleArray*>(data);
  vtkDoubleArray* doubleData = vtkDoubleArray::SafeDownCast (data);
  
  if(doubleData)
    {
    cout << "doubleData is valid" << endl;
    }
  
  for(unsigned int i = 0; i < doubleData->GetNumberOfTuples(); i++)
    {
    double val = doubleData->GetValue(i);
    }
  
	return EXIT_SUCCESS;
}