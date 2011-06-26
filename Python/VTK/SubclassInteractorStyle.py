#!/usr/bin/python

import vtk

class SubclassInteractor(vtk.vtkInteractorStyleTrackballActor):
	def OnLeftButtonDown(self):
		print "LeftButtonDown"

print "Start"

# create a rendering window and renderer
ren = vtk.vtkRenderer()
renWin = vtk.vtkRenderWindow()
renWin.AddRenderer(ren)
 
# create a renderwindowinteractor
iren = vtk.vtkRenderWindowInteractor()
iren.SetRenderWindow(renWin)
 
style = SubclassInteractor()
iren.SetInteractorStyle(style)
 
# create source
sphereSource = vtk.vtkSphereSource()
sphereSource.Update()
  
# mapper
mapper = vtk.vtkPolyDataMapper()
mapper.SetInput(sphereSource.GetOutput())
 
# actor
actor = vtk.vtkActor()
actor.SetMapper(mapper)

# assign actor to the renderer
ren.AddActor(actor)
 
# enable user interface interactor
iren.Initialize()
renWin.Render()
iren.Start()
