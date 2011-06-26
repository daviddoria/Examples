import math, os, sys
import wx
import vtk
from vtk.wx.wxVTKRenderWindowInteractor import wxVTKRenderWindowInteractor

# every wx app needs an app
app = wx.PySimpleApp()

# create the top-level frame, sizer and wxVTKRWI
frame = wx.Frame(None, -1, "wxRenderWindow", size=wx.Size(400,400))

# Add the wxVTKRenderWindowInteractor
widget = wxVTKRenderWindowInteractor(frame, -1)
widget.Enable(1)

# Add a VTKRenderer
ren = vtk.vtkRenderer()
widget.GetRenderWindow().AddRenderer(ren)

# You need to ad a sizer so that the widget will fit the frame
sizer = wx.BoxSizer(wx.VERTICAL)
sizer.Add(widget, 1, wx.EXPAND)
frame.SetSizer(sizer)
frame.Layout()

# Now make the frame show
frame.Show()

# strat the app
app.MainLoop()

