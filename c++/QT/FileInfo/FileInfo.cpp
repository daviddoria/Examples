#include <iostream>

#include <QFileInfo>

int main(int, char *[])
{
  QFileInfo myFile("CMakeLists.txt");
  if(myFile.exists())
    {
    std::cout << "File exists" << std::endl;
    std::cout << "fileName: " << myFile.fileName().toStdString() << std::endl;
    std::cout << "filePath: " << myFile.filePath().toStdString() << std::endl;
    std::cout << "absolutePath: " << myFile.absolutePath().toStdString() << std::endl;
    std::cout << "baseName: " << myFile.baseName().toStdString() << std::endl;
    std::cout << "extension: " << myFile.suffix().toStdString() << std::endl;
    }
  else
    {
    std::cout << "File does not exist" << std::endl;
    return 0;
    }


  return 0;
}
