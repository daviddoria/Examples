import vtk

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
Colors = vtk.vtkUnsignedCharArray()
Colors.SetNumberOfComponents(3)
Colors.SetName('Colors')
Colors.InsertNextTuple3( 255,0,0 )
Colors.InsertNextTuple3( 0,255,0)
Colors.InsertNextTuple3( 0,0,255 )
 
polydata = vtk.vtkPolyData()
polydata.SetPoints(Points)
polydata.SetPolys(Triangles)
polydata.GetPointData().SetScalars(Colors)
polydata.Modified()
polydata.Update()

mapper = vtk.vtkPolyDataMapper()
mapper.SetInput(polydata)
actor = vtk.vtkActor()
actor.SetMapper(mapper)

ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)
ren.AddActor(actor)
iren.Start()
