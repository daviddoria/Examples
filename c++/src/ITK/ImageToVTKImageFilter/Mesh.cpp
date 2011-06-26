#include "itkMesh.h"

int main(int argc, char **argv)
{
	typedef itk::Image<unsigned short, 2> ImageType;
	typedef itk::ImageFileReader<ImageType> ReaderType;
	typedef itk::ImageToVTKImageFilter<ImageType> ConnectorType;
	
	ReaderType::Pointer reader = ReaderType::New();
	ConnectorType::Pointer connector = ConnectorType::New();
	
	reader->SetFileName(argv[1]);
	connector->SetInput(reader->GetOutput());
	
	vtkImageViewer* viewer = vtkImageViewer::New();
	
	vtkRenderWindowInteractor* renderWindowInteractor = vtkRenderWindowInteractor::New();
	
	viewer->SetupInteractor(renderWindowInteractor);
	viewer->SetInput(connector->GetOutput());
	
	viewer->Render();
	viewer->SetColorWindow(255);
	viewer->SetColorLevel(128);
	renderWindowInteractor->Start();
	
}