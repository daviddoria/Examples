#include <iostream>
#include <cstdlib> //for system()

int main(int argc, char *argv[])
{
  std::string Command = "ls";
  system(Command.c_str());
  return 0;
}
