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

  // Check if a directory named "Folder" exists use:
  QDir("Folder").exists();

  // Create a new folder named "MyFolder" use:
  QDir().mkdir("MyFolder");

  // Relative paths
  QDir homeDir("/home/doriad");
  std::cout << homeDir.relativeFilePath("test.jpg").toStdString() << std::endl;
  std::cout << homeDir.relativeFilePath("/test.jpg").toStdString() << std::endl;

  std::cout << homeDir.filePath("/test").toStdString() << std::endl;
  std::cout << homeDir.filePath("test").toStdString() << std::endl;

  // Get working directory
  std::cout << QDir::currentPath().toStdString() << std::endl;

  // Set working directory
  std::cout << QDir::setCurrent("/home/doriad") << std::endl;
  std::cout << QDir::currentPath().toStdString() << std::endl;
  std::cout << QDir::setCurrent(QDir::current().filePath("Debug")) << std::endl;
  std::cout << QDir::currentPath().toStdString() << std::endl;
  return 0;
}
