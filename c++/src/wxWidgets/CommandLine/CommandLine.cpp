#include "wx/string.h"
#include <wx/cmdline.h>
#include <iostream>

#include "CommandLineOptions.h"

// http://wiki.wxwidgets.org/Command-Line_Arguments

int main()
{
  std::cout << wxCMD_LINE_VAL_STRING << " " << wxCMD_LINE_VAL_NUMBER << std::endl;
  int a = wxCMD_LINE_VAL_STRING;
  return 0;
}
