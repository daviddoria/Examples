#!/usr/bin/python


import wx 
# always add this line, or your code won't compile - the wx module contains the GUI code you'll use

#The start of our wxPython GUI tutorial"""

app = wx.App(redirect=False) # create a wx application

window = wx.Frame(None, title = 'Sample GUI App') # create a window
btn = wx.Button(window) 
# create a button 'widget' on the window - note how the button receives the window. This creates a tree-like structure where the window is the parent node and anything else like buttons are child nodes.

window.Show() # make the frame (and hence button) visible
app.MainLoop() # keep things going
