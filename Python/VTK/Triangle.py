import vtk
from vtk import *
import hybrid
from hybrid import *

Points = vtk.vtkPoints()
Triangles = vtk.vtkCellArray()
Triangle = vtk.vtkTriangle();

id = Points.InsertNextPoint(1.0, 0.0, 0.0)
id = Points.InsertNextPoint(0.0, 0.0, 0.0)
id = Points.InsertNextPoint(0.0, 1.0, 0.0)

Triangle.GetPointIds().SetId(0, 0);
Triangle.GetPointIds().SetId(1, 1);
Triangle.GetPointIds().SetId(2, 2);
Triangles.InsertNextCell(Triangle);


polydata = vtk.vtkPolyData()
polydata.SetPoints(Points)
polydata.SetPolys(Triangles)
polydata.Modified()
polydata.Update()

writer = vtk.vtkXMLPolyDataWriter();
writer.SetFileName("Triangle.vtp");
writer.SetInput(polydata);
writer.Write();

