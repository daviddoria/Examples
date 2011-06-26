#include "wx/filename.h"

#include <iostream>
#include <string>

int main()
{
  wxFileName dirname( "/home/doriad/test.jpg");

  std::string extension(dirname.GetExt().GetData());
  if(extension.compare("jpg") == 0)
    {
    std::cout << "File is a jpg!" << std::endl;
    }
  else
    {
    std::cout << "File is NOT a jpg!" << std::endl;
    std::cout << dirname.GetExt() << std::endl;
    }

  //std::cout << dirname.GetPath() << std::endl;

  dirname.SetExt("png");

  std::cout << dirname.GetFullName().ToAscii() << std::endl;

  return 0;
}
