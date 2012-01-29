#include <QHash>

#include <iostream>

int main()
{
  QHash<QString, int> hash;
  
  hash["one"] = 1;
  hash["three"] = 3;
  hash["seven"] = 7;

  int num1 = hash["seven"];
  std::cout << "num1: " << num1 << std::endl;

  return 0;
}