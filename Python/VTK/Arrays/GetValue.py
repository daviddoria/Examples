import vtk
from vtk import *

#setup sphere
sphereSource = vtk.vtkSphereSource()
sphereSource.Update()

polydata = vtk.vtkPolyData()
polydata.ShallowCopy(sphereSource.GetOutput())

normals = polydata.GetPointData().GetNormals();
normal0 = normals.GetTuple3(0);

print "Normal0: " + str(normal0[0]) + " " + str(normal0[1]) + " " + str(normal0[2])
