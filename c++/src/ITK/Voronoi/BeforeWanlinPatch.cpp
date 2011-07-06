#include "itkVoronoiDiagram2DGenerator.h"
#include "itkImageFileWriter.h"
#include "itkVTKPolyDataWriter.h"

int main(int, char* [] )
{
  const double height = 100;
  const double width = 100;
  
  typedef itk::VoronoiDiagram2D<double> VoronoiDiagramType;
  typedef itk::VoronoiDiagram2DGenerator<double> VoronoiGeneratorType;

  typedef VoronoiDiagramType::PointType PointType;
  typedef VoronoiDiagramType::CellType CellType;
  typedef VoronoiDiagramType::CellAutoPointer CellAutoPointer;
  typedef CellType::PointIdIterator PointIdIterator;
  typedef VoronoiDiagramType::NeighborIdIterator NeighborIdIterator;

  VoronoiDiagramType::Pointer voronoiDiagram = VoronoiDiagramType::New();
  VoronoiGeneratorType::Pointer voronoiGenerator = VoronoiGeneratorType::New();

  PointType insize;
  insize[0] = width;
  insize[1] = height;
  voronoiGenerator->SetBoundary(insize);

  // Create a list of seeds
  std::vector<PointType> seeds;
  PointType seed0;
  seed0[0] = 50;
  seed0[1] = 50;
  seeds.push_back(seed0);
  
  PointType seed1;
  seed1[0] = 25;
  seed1[1] = 25;
  seeds.push_back(seed1);
  
  PointType seed2;
  seed2[0] = 75;
  seed2[1] = 25;
  seeds.push_back(seed2);
  
  PointType seed3;
  seed3[0] = 25;
  seed3[1] = 75;
  seeds.push_back(seed3);
  
  PointType seed4;
  seed4[0] = 75;
  seed4[1] = 75;
  seeds.push_back(seed4);
  
  for(unsigned int i = 0; i < seeds.size(); ++i)
    {
    voronoiGenerator->AddOneSeed(seeds[i]);
    }
  
  voronoiGenerator->Update();
  voronoiDiagram = voronoiGenerator->GetOutput();

  for(unsigned int i = 0; i < seeds.size(); i++)
    {
    PointType currP = voronoiDiagram->GetSeed(i);
    std::cout << "Seed No." << i << ": At (" << currP[0] << "," << currP[1] << ")" << std::endl;
    std::cout << "  Boundary Vertices List (in order):";
    CellAutoPointer currCell;
    voronoiDiagram->GetCellId(i, currCell);
    PointIdIterator currCellP;
    for(currCellP = currCell->PointIdsBegin(); currCellP != currCell->PointIdsEnd(); ++currCellP)
      {
      std::cout << (*currCellP) << ",";
      }
    std::cout << std::endl;
    std::cout << "  Neighbors (Seed No.):";
    NeighborIdIterator currNeibor;
    for(currNeibor = voronoiDiagram->NeighborIdsBegin(i); currNeibor != voronoiDiagram->NeighborIdsEnd(i); ++currNeibor)
      {
      std::cout << (*currNeibor) << ",";
      }
    std::cout << std::endl << std::endl;
    }

  std::cout << "Vertices Informations:" << std::endl;
  VoronoiDiagramType::VertexIterator allVerts;
  int j = 0;
  for(allVerts = voronoiDiagram->VertexBegin(); allVerts != voronoiDiagram->VertexEnd(); ++allVerts)
    {
    voronoiDiagram->SetPoint(j, *allVerts);
    std::cout << "Vertices No." << j;
    j++;
    std::cout << ": At (" << (*allVerts)[0] << "," << (*allVerts)[1] << ")" << std::endl;
    }

  // Write the resulting mesh
  typedef itk::VTKPolyDataWriter<VoronoiDiagramType::Superclass> WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetInput(voronoiDiagram);
  writer->SetFileName("voronoi.vtk");
  writer->Update();

  // Setup an image to visualize the input
  {
  typedef itk::Image< unsigned char, 2>  ImageType;

  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start,size);

  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  ImageType::IndexType ind;
  ind[0] = 50;
  ind[1] = 50;
  image->SetPixel(ind, 255);

  ind[0] = 25;
  ind[1] = 25;
  image->SetPixel(ind, 255);

  ind[0] = 75;
  ind[1] = 25;
  image->SetPixel(ind, 255);

  ind[0] = 25;
  ind[1] = 75;
  image->SetPixel(ind, 255);
  
  ind[0] = 75;
  ind[1] = 75;
  image->SetPixel(ind, 255);

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("image.png");
  writer->SetInput(image);
  writer->Update();
  }

  return EXIT_SUCCESS;
}
