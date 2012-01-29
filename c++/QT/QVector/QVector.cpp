#include <QVector>

#include <iostream>

int main()
{
  QVector<int> a;

  a.push_back(1);

  std::cout << a[0] << std::endl;
  return 0;
}