#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char *argv[])
{

  std::string hexNumber = "0x12";
  std::stringstream ss;
  ss << hexNumber;
  int output;
  ss >> std::hex >> output;
  std::cout << output;
  return 0;
}
