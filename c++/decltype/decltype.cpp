#include <iostream>

int main(int argc, char *argv[])
{
  int x = 3;
  decltype(x) y = x; // same thing as auto y = x; or in this case int y = 3;
  return 0;
}