import vtk
from vtk import *
import hybrid
from hybrid import *

#setup points and vertices
Points = vtk.vtkPoints()
Triangles = vtk.vtkCellArray()

Points.InsertNextPoint(1.0, 0.0, 0.0)
Points.InsertNextPoint(0.0, 0.0, 0.0)
Points.InsertNextPoint(0.0, 1.0, 0.0)

Triangle = vtk.vtkTriangle();
Triangle.GetPointIds().SetId(0, 0);
Triangle.GetPointIds().SetId(1, 1);
Triangle.GetPointIds().SetId(2, 2);
Triangles.InsertNextCell(Triangle);

#setup colors
Colors = vtk.vtkUnsignedCharArray();
Colors.SetNumberOfComponents(3);
Colors.SetName("Colors");
Colors.InsertNextTuple3(255,0,0);

polydata = vtk.vtkPolyData()
polydata.SetPoints(Points)
polydata.SetPolys(Triangles)

polydata.GetCellData().SetVectors(Colors);
polydata.Modified()
polydata.Update()

writer = vtk.vtkXMLPolyDataWriter();
writer.SetFileName("TriangleSolidColor.vtp");
writer.SetInput(polydata);
writer.Write();
