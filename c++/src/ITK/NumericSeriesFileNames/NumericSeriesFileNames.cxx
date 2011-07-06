#include "itkNumericSeriesFileNames.h"

int main(int, char *[])
{
  itk::NumericSeriesFileNames::Pointer numericSeriesFileNames = itk::NumericSeriesFileNames::New();
  numericSeriesFileNames->SetStartIndex(5);
  numericSeriesFileNames->SetEndIndex(20);
  numericSeriesFileNames->SetIncrementIndex(2);
  numericSeriesFileNames->SetSeriesFormat("output_%d.png");
  
  std::vector< std::string > fileNames = numericSeriesFileNames->GetFileNames();
  
  for(unsigned int i = 0; i < fileNames.size(); ++i)
    {
    std::cout << fileNames[i] << std::endl;
    }

  return EXIT_SUCCESS;
}
