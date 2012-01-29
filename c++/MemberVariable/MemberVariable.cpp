#include <iostream>

class MyClass
{
private:
  double a;
};

int main(int argc, char *argv[])
{
  unsigned int number = 5;
  std::string test = ZeroPad(number, 3);
  std::cout << test << std::endl;

  return 0;
}

std::string ZeroPad(const unsigned int number, const unsigned int rep)
{
  std::stringstream Padded;
  Padded << std::setfill('0') << std::setw(rep) << number;

  return Padded.str();
}
