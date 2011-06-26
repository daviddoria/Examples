'''
Created on Jun 15, 2009

@author: ssnyders
'''

import vtk
from vtk import *
import hybrid
from hybrid import *

# ============ create source points ==============
print "Creating source points..."
sourcePoints = vtk.vtkPoints()
sourceVertices = vtk.vtkCellArray()

id = sourcePoints.InsertNextPoint(1.0, 0.1, 0.0)
sourceVertices.InsertNextCell(1)
sourceVertices.InsertCellPoint(id)

id = sourcePoints.InsertNextPoint(0.1, 1.1, 0.0)
sourceVertices.InsertNextCell(1)
sourceVertices.InsertCellPoint(id)

id = sourcePoints.InsertNextPoint(0.0, 0.1, 1.0)
sourceVertices.InsertNextCell(1)
sourceVertices.InsertCellPoint(id)

source = vtk.vtkPolyData()
source.SetPoints(sourcePoints)
source.SetVerts(sourceVertices)
source.Update()

print "Displaying source points..."
# ============ display source points ==============
pointCount = 3
index = 0
while index < pointCount:
    point = [0,0,0]
    sourcePoints.GetPoint(index, point)
    print "source point[%s]=%s" % (index,point)
    index += 1

#============ create target points ==============
print "Creating target points..."
targetPoints = vtk.vtkPoints()
targetVertices = vtk.vtkCellArray()

id = targetPoints.InsertNextPoint(1.0, 0.0, 0.0)
targetVertices.InsertNextCell(1)
targetVertices.InsertCellPoint(id)

id = targetPoints.InsertNextPoint(0.0, 1.0, 0.0)##
targetVertices.InsertNextCell(1)
targetVertices.InsertCellPoint(id)

id = targetPoints.InsertNextPoint(0.0, 0.0, 1.0)
targetVertices.InsertNextCell(1)
targetVertices.InsertCellPoint(id)

target = vtk.vtkPolyData()
target.SetPoints(targetPoints)
target.SetVerts(targetVertices)
target.Update()

# ============ display target points ==============
print "Displaying target points..."
pointCount = 3
index = 0
while index < pointCount:
    point = [0,0,0]
    targetPoints.GetPoint(index, point)
    print "target point[%s]=%s" % (index,point)
    index += 1

print "Running ICP ----------------"
# ============ run ICP ==============
icp = vtk.vtkIterativeClosestPointTransform()
icp.SetSource(source)
icp.SetTarget(target)
icp.GetLandmarkTransform().SetModeToRigidBody()
#icp.DebugOn()
icp.SetMaximumNumberOfIterations(20)
icp.StartByMatchingCentroidsOn()
icp.Modified()
icp.Update()

icpTransformFilter = vtk.vtkTransformPolyDataFilter()
icpTransformFilter.SetInput(source)
icpTransformFilter.SetTransform(icp)
icpTransformFilter.Update()

transformedSource = icpTransformFilter.GetOutput()

# ============ display transformed points ==============
pointCount = 3
index = 0
while index < pointCount:
    point = [0,0,0]
    transformedSource.GetPoint(index, point)
    print "xformed source point[%s]=%s" % (index,point)
    index += 1

