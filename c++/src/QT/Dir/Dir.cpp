#include <iostream>

#include <QDir>

int main(int, char *[])
{
  std::cout << "Home: " << QDir::home().path().toStdString() << std::endl;
  std::cout << "Separator: " << QDir::separator().toAscii() << std::endl;

  // Test append filename to directory
  QString string1 = "/home/doriad";
  QString string2 = "/home/doriad/";

  std::cout << QDir(string1).absoluteFilePath("test.txt").toStdString() << std::endl;
  std::cout << QDir(string2).absoluteFilePath("test.txt").toStdString() << std::endl;

  std::cout << "User home path: " << QDir::homePath().toStdString() << std::endl;
  
  return 0;
}
