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

