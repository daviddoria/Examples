#include <QColor>

#include <iostream>

int main(int argc, char *argv[])
{

  QColor a("blue");
  std::cout << a.blue() << std::endl;

  return 0;
}
