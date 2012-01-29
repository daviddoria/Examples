#include <QColor>

#include <iostream>

int main(int argc, char *argv[])
{

  QColor blue("blue");
  std::cout << blue.red() << " " << blue.green() << " " << blue.blue() << std::endl;
  
  QColor blue2(Qt::blue);
  std::cout << blue2.red() << " " << blue2.green() << " " << blue2.blue() << std::endl;

  return 0;
}
