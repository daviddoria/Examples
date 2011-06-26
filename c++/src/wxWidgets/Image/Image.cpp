#include "wx/image.h"
#include <iostream>

int main()
{
  wxImage myImage;
  std::cout << "Exists? " << wxFileExists(wxString("/media/portable/src/image-completer-gcc/test-data/elephant-input.png")) << std::endl;
  myImage.LoadFile("/media/portable/src/image-completer-gcc/test-data/elephant-input.png");
  return 0;
}
