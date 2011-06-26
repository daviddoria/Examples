import vtk
from vtk import *
import hybrid
from hybrid import *

#setup points and vertices
Points = vtk.vtkPoints()
Vertices = vtk.vtkCellArray()

id = Points.InsertNextPoint(1.0, 0.0, 0.0)
Vertices.InsertNextCell(1)
Vertices.InsertCellPoint(id)
id = Points.InsertNextPoint(0.0, 0.0, 0.0)
Vertices.InsertNextCell(1)
Vertices.InsertCellPoint(id)
id = Points.InsertNextPoint(0.0, 1.0, 0.0)
Vertices.InsertNextCell(1)
Vertices.InsertCellPoint(id)

#setup colors
Colors = vtk.vtkUnsignedCharArray();
Colors.SetNumberOfComponents(3);
Colors.SetName("Colors");
Colors.InsertNextTuple3(255,0,0);
Colors.InsertNextTuple3(0,255,0);
Colors.InsertNextTuple3(0,0,255);

polydata = vtk.vtkPolyData()
polydata.SetPoints(Points)
polydata.SetVerts(Vertices)
polydata.GetPointData().SetVectors(Colors);
polydata.Modified()
polydata.Update()

writer = vtk.vtkXMLPolyDataWriter();
writer.SetFileName("TriangleColoredPoints.vtp");
writer.SetInput(polydata);
writer.Write();
