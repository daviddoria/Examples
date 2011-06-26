#include "itkPointSet.h"

int main( int argc, char* argv[] )
{
  // Verify the number of parameters in the command line
  if( argc < 1 )
    {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << " outputFile " << std::endl;
    return EXIT_FAILURE;
    }
  
  std::string outputFilename = argv[1];
  
  // Store points
  typedef itk::PointSet<double, 3 > PointSetType;
  PointSetType::Pointer pointsSet = PointSetType::New();
  typedef PointSetType::PointType PointType;
  
      
  return EXIT_SUCCESS;
}