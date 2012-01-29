#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#ifdef _DEBUG
	#pragma comment(lib,"vtkCommon.lib")
	#pragma comment(lib,"vtkDICOMParser.lib")
	#pragma comment(lib,"vtkFiltering.lib")
	#pragma comment(lib,"vtkGenericFiltering.lib")
	#pragma comment(lib,"vtkGraphics.lib")
	#pragma comment(lib,"vtkHybrid.lib")
	#pragma comment(lib,"vtkIO.lib")
	#pragma comment(lib,"vtkImaging.lib")	
	#pragma comment(lib,"vtkNetCDF.lib")
	#pragma comment(lib,"vtkRendering.lib")
	#pragma comment(lib,"vtkVolumeRendering.lib")
	#pragma comment(lib,"vtkWidgets.lib")
	#pragma comment(lib,"vtkexoIIc.lib")
	#pragma comment(lib,"vtkexpat.lib")
	#pragma comment(lib,"vtkfreetype.lib")
	#pragma comment(lib,"vtkftgl.lib")
	#pragma comment(lib,"vtkjpeg.lib")
	#pragma comment(lib,"vtkpng.lib")
	#pragma comment(lib,"vtksys.lib")
	#pragma comment(lib,"vtktiff.lib")
	#pragma comment(lib,"vtkzlib.lib")
	#pragma comment(lib,"opengl32.lib")
	#pragma comment(lib,"glu32.lib")

#else // _RELEASE
	#pragma comment(lib,"vtkCommon.lib")
	#pragma comment(lib,"vtkDICOMParser.lib")
	#pragma comment(lib,"vtkFiltering.lib")
	#pragma comment(lib,"vtkGenericFiltering.lib")
	#pragma comment(lib,"vtkGraphics.lib")
	#pragma comment(lib,"vtkHybrid.lib")
	#pragma comment(lib,"vtkIO.lib")
	#pragma comment(lib,"vtkImaging.lib")	
	#pragma comment(lib,"vtkNetCDF.lib")
	#pragma comment(lib,"vtkRendering.lib")
	#pragma comment(lib,"vtkVolumeRendering.lib")
	#pragma comment(lib,"vtkWidgets.lib")
	#pragma comment(lib,"vtkexoIIc.lib")
	#pragma comment(lib,"vtkexpat.lib")
	#pragma comment(lib,"vtkfreetype.lib")
	#pragma comment(lib,"vtkftgl.lib")
	#pragma comment(lib,"vtkjpeg.lib")
	#pragma comment(lib,"vtkpng.lib")
	#pragma comment(lib,"vtksys.lib")
	#pragma comment(lib,"vtktiff.lib")
	#pragma comment(lib,"vtkzlib.lib")
	#pragma comment(lib,"opengl32.lib")
	#pragma comment(lib,"glu32.lib")
#endif

#define VTK_DATA_ROOT "D:\\Users\\Peter\\bin\\VTKData\\Data\\"  

#endif // PROJECT_CONFIG_H