#include <vtkIntArray.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLPolyDataReader.h>


void ExtraDataPerPoint(const std::string &OutputFilename);
void ExtraDataPerCell(const std::string &OutputFilename);
void ExtraDataSingle(const std::string &OutputFilename);

void GetExtraDataSingle(const std::string &InputFilename);

void GetUnknownArrayType(const std::string &InputFilename, const std::string &ArrayName);

unsigned char r[3];
unsigned char g[3];
unsigned char b[3];

double Origin[3];
double X[3];	
double Y[3];
double Z[3];

void SetGlobals();

int main(int argc, char *argv[])
{
	SetGlobals();

	//ExtraDataPerPoint("TestUnknownArray.vtp");
	ExtraDataPerCell("TestCellData.vtp");
	//ExtraDataSingle("ExtraDataSingle.vtp");
	//GetExtraDataSingle("ExtraDataSingle.vtp");
	
	//GetUnknownArrayType("TestUnknownArray.vtp", "Colors"); //unsigned char
	//GetUnknownArrayType("TestUnknownArray.vtp", "PositionIndex"); //int
			
	return 0;
}

void GetUnknownArrayType(const std::string &InputFilename, const std::string &ArrayName)
{
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(InputFilename.c_str());
  reader->Update();
  vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();

  
  vtkSmartPointer<vtkDoubleArray> Array = vtkDoubleArray::SafeDownCast(polydata->GetPointData()->GetArray(ArrayName.c_str()));
  
  if(Array)
    {
    cout << "Got array." << endl;
    double Location[3];
    Array->GetTupleValue(0, Location);
    cout << "Location: " << Location[0] << ","  << Location[1] << "," << Location[2] << endl;
    }
  else
    {
    cout << "no array." << endl;
    }
}

void ExtraDataPerPoint(const std::string &OutputFilename)
{

  //setup VTK things
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New(); 
  
  //setup position index
  vtkSmartPointer<vtkIntArray> PositionIndex = vtkSmartPointer<vtkIntArray>::New();
  PositionIndex->SetNumberOfComponents(1);
  PositionIndex->SetName("PositionIndex");
  PositionIndex->InsertNextValue(10);
  PositionIndex->InsertNextValue(20);
  PositionIndex->InsertNextValue(30);
  
  //setup colors
  vtkSmartPointer<vtkCellArray> Vertices = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkUnsignedCharArray> Colors = vtkSmartPointer<vtkUnsignedCharArray>::New();

  Colors->SetNumberOfComponents(3);
  Colors->SetName("Colors");
  
  Colors->InsertNextTupleValue(r);
  Colors->InsertNextTupleValue(g);
  Colors->InsertNextTupleValue(b);
  
  vtkIdType locx = pts->InsertNextPoint(X);
  Vertices->InsertNextCell(1, &locx);
      
  vtkIdType locy = pts->InsertNextPoint(Y);
  Vertices->InsertNextCell(1, &locy);
      
  vtkIdType locz = pts->InsertNextPoint(Z);
  Vertices->InsertNextCell(1, &locz);
  
  vtkSmartPointer<vtkPolyData> pdata = vtkSmartPointer<vtkPolyData>::New();

  //add the points to the dataset
  pdata->SetPoints(pts);
  pdata->SetVerts(Vertices);


  pdata->GetPointData()->AddArray(Colors);
  pdata->GetPointData()->AddArray(PositionIndex);
  
  //write the file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(pdata);

  writer->SetFileName(OutputFilename.c_str());
  writer->Write();
	
}


void ExtraDataPerCell(const std::string &OutputFilename)
{
	//setup points
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	points->InsertNextPoint(1.0, 0.0, 0.0);
	points->InsertNextPoint(0.0, 0.0, 0.0);
	points->InsertNextPoint(0.0, 1.0, 0.0);
	
	//setup triangle
    vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
	triangle->GetPointIds()->SetId(0,0);
	triangle->GetPointIds()->SetId(1,1);
	triangle->GetPointIds()->SetId(2,2);
	triangles->InsertNextCell(triangle);
	
	//setup data for the triangle
    vtkSmartPointer<vtkDoubleArray> TriangleArea = vtkSmartPointer<vtkDoubleArray>::New();
	TriangleArea->SetNumberOfComponents(1);
	TriangleArea->SetName("TriangleArea");
	TriangleArea->InsertNextValue(1.45);
	
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata->SetPoints(points);
	polydata->SetPolys(triangles);
	polydata->GetCellData()->AddArray(TriangleArea);
	
	//write the file
    vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
	writer->SetInput(polydata);

	writer->SetFileName(OutputFilename.c_str());
	writer->Write();
	
}

void GetExtraDataSingle(const std::string &InputFilename)
{
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  cout << "Reading " << InputFilename << endl;
  reader->SetFileName(InputFilename.c_str());
  reader->Update();
  vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();

  vtkSmartPointer<vtkDoubleArray> locationArray = vtkDoubleArray::SafeDownCast(polydata->GetFieldData()->GetArray("Location"));
  
  if(locationArray)
    {
    cout << "Got array." << endl;
    double location[3];
    locationArray->GetTupleValue(0, location);
    cout << "Location: " << location[0] << ","  << location[1] << "," << location[2] << endl;
    }
  else
    {
    cout << "no location." << endl;
    }
}

void ExtraDataSingle(const std::string &OutputFilename)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
  for ( unsigned int i = 0; i < 4; ++i )
    {
    vtkIdType pid[1];
    pid[0] = points->InsertNextPoint(drand48(), drand48(), drand48());
    vertices->InsertNextCell(1,pid);
    }
  
  vtkSmartPointer<vtkPolyData> pdata = vtkSmartPointer<vtkPolyData>::New();

  //add the points to the dataset
  pdata->SetPoints(points);
  pdata->SetVerts(vertices);

  vtkSmartPointer<vtkIntArray> myArray = vtkSmartPointer<vtkIntArray>::New();
  myArray->SetNumberOfComponents(1);
  myArray->InsertNextValue(4);
  myArray->SetName("FileIndex");

  pdata->GetFieldData()->AddArray(myArray);

  vtkSmartPointer<vtkDoubleArray> location = vtkSmartPointer<vtkDoubleArray>::New();
  double loc[3] = {10.0, 20.0, 30.0};
  location->SetNumberOfComponents(3);
  location->SetName("Location");
  location->InsertNextTuple(loc);
  
  pdata->GetFieldData()->AddArray(location);
  
  //write the file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(pdata);
  writer->SetFileName(OutputFilename.c_str());
  writer->Write();
  cout << "Wrote " << OutputFilename.c_str() << endl;
}

void SetGlobals()
{
  Origin[0] = 0;
  Origin[1] = 0;
  Origin[2] = 0;
  
  X[0] = 1;
  X[1] = 0;
  X[2] = 0;
      
  Y[0] = 0;
  Y[1] = 1;
  Y[2] = 0;
  
  Z[0] = 0;
  Z[1] = 0;
  Z[2] = 1;

  /*
  Color<unsigned char> R = Red();
  r[0] = R.getR();
  r[1] = R.getG();
  r[2] = R.getB();
  
  Color<unsigned char> G = Green();
  g[0] = G.getR();
  g[1] = G.getG();
  g[2] = G.getB();

  Color<unsigned char> B = Blue();
  b[0] = B.getR();
  b[1] = B.getG();
  b[2] = B.getB();
  */
}
