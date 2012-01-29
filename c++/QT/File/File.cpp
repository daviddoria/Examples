#include <iostream>

#include <QFile>

// To do this sort of operation with directories, see QDir.

int main(int, char *[])
{
  QFile myFile("test.txt");
  if(myFile.exists())
    {
    std::cout << "File exists" << std::endl;
    }
  else
    {
    std::cout << "File does not exist" << std::endl;
    return 0;
    }

  if(myFile.open(QIODevice::ReadOnly))
    {
    std::cout << "opened successfully" << std::endl;
    }
  else
    {
    std::cout << "failed to open" << std::endl;
    }

  return 0;
}
