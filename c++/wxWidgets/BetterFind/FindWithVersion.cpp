#include "wx/string.h"
#include <iostream>

void TestBasename();

int main()
{
  wxString test1 = wxString::FromAscii("ii");

  // Problem 1 - no matching function Format(wxChar*, ....)

  wxString test2 = wxString::Format(wxT("\n%s%s\n"), "test");
  std::cout << test2 << std::endl;

  // Problem 2 - cannot convert to char*
  //const char* a = test2.c_str();
  //const char* a = test2.GetData();
  const char* a = test2.mb_str();

  // Problem 3 - no operator+= defined
  wxString test3(wxString::FromAscii("test"));
  test3 += wxString::FromAscii("world");

  TestBasename();

  return 0;
}

void TestBasename()
{
  wxString originalString = wxString::FromAscii("test.jpg");
  wxString base = originalString.BeforeLast('.');
  std::cout << "original string: " << originalString.ToAscii() << std::endl;
  std::cout << "basename: " << base.ToAscii() << std::endl;
}