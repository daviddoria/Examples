#include <itkIndex.h>
#include <set>

#include "IndexComparison.h"

int main(int, char*[])
{
  itk::Index<2> pixel;
  pixel[0] = 10;
  pixel[1] = 10;

  itk::Index<2> pixel2;
  pixel2[0] = 10;
  pixel2[1] = 11;

  typedef std::set<itk::Index<2>, IndexComparison > SetType;
  SetType s;
  std::pair<SetType::iterator, bool> result = s.insert(pixel);
  std::cout << result.second << std::endl;

  result = s.insert(pixel2);
  std::cout << result.second << std::endl;

  return EXIT_SUCCESS;
}
