# -*- coding: utf-8 -*-

import wx
from vtk.wx.wxVTKRenderWindowInteractor import wxVTKRenderWindowInteractor
import vtk
import numpy as N
import os

#----------------------------------------------------------------------------------------------
def numpy2vtk(numpyArray):
  """
  Convert a numpy double array to a vtkDoubleArray
  """
  vtkArray = vtk.vtkDoubleArray()
  if numpyArray.ndim < 2:
    vtkArray.SetNumberOfComponents(1)
    vtkArray.SetNumberOfTuples(numpyArray.size)
  else:
    vtkArray.SetNumberOfComponents(numpyArray.T.shape[0])
    vtkArray.SetNumberOfTuples(numpyArray.size/numpyArray.T.shape[0])
  vtkArray.SetVoidArray(numpyArray, numpyArray.size, 1)
  return vtkArray

#----------------------------------------------------------------------------------------------  
class MyFrame(wx.Frame):
  def __init__(self, id):
    wx.Frame.__init__(self, id, title='My first VTKwx example', style =wx.DEFAULT_FRAME_STYLE)

    ## Add a menubar
    menuBar = wx.MenuBar()
    menuF = wx.Menu()
    menuBar.Append(menuF, "File")
    self.draw = menuF.Append(wx.ID_OPEN, 'Draw') 
    self.SetMenuBar(menuBar)

    self.sb = self.CreateStatusBar()
    self.sb.SetFieldsCount(2)

    # Add the vtk window widget
    self.widget = wxVTKRenderWindowInteractor(self, -1)
    self.widget.Enable(1)

    # Layout
    sizer = wx.BoxSizer(wx.VERTICAL)
    sizer.Add(self.widget, 1, wx.EXPAND)
    self.SetSizer(sizer)
    self.Layout()

    # Ad a renderer
    self.ren = vtk.vtkRenderer()
    self.widget.GetRenderWindow().AddRenderer(self.ren)
    
    # Bind the menu
    self.Bind(wx.EVT_MENU, self.onDraw, self.draw)

  def onDraw(self, evt):
    """ Draw some random points on the screen. """

    # Some random points as numpy array
    self.xyz = N.random.random((1000000, 3))

    # Convert numpy array to a VTK array
    self.xyzVTK = numpy2vtk(self.xyz)
    
    # Create points
    points = vtk.vtkPoints()
    points.SetData(self.xyzVTK)
    
    # Make polydata  from points
    pd = vtk.vtkPolyData()
    pd.SetPoints(points)
    
    # Point must be transformed to cells to be plotted. vtkMaskPoints can do it for us
    mask = vtk.vtkMaskPoints()
    mask.SetInput(pd)
    mask.GenerateVerticesOn()
    mask.SetOnRatio(1)

    # Define a mapper
    m = vtk.vtkPolyDataMapper()
    m.SetInput(mask.GetOutput())
    
    # Define an actor
    actor = vtk.vtkActor()
    actor.SetMapper(m)
    
    # Add the actor to the vtkRenderer
    self.ren.AddActor(actor)
    
    # Reset the camera to view the dataset
    self.ren.ResetCamera()

    # Render the scene
    self.widget.Render()
    
#----------------------------------------------------------------------------------------------
if __name__ == "__main__":
  app = wx.App(0)
  frame = MyFrame(None)
  frame.Show()
  app.MainLoop()

