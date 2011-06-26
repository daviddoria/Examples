import vtk
from vtk import *
import hybrid
from hybrid import *

Points = vtk.vtkPoints()
Points.InsertNextPoint(1.0, 0.0, 0.0)
Points.InsertNextPoint(0.0, 0.0, 0.0)
Points.InsertNextPoint(0.0, 1.0, 0.0)

polydata = vtk.vtkPolyData()
polydata.SetPoints(Points)
polydata.Update()

writer = vtk.vtkXMLPolyDataWriter();
writer.SetFileName("TrianglePoints.vtp");
writer.SetInput(polydata);
writer.Write();

