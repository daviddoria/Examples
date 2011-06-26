#include <vtkSmartPointer.h>
#include <vtkDirectory.h>

#include <vtksys/SystemTools.hxx>

int main(int argc, char *argv[])
{
  if(argc != 2)
    {
    std::cout << "Required arguments: FolderName" << std::endl;
    return EXIT_FAILURE;
    }

  std::string DirectoryName = argv[1];
  vtkSmartPointer<vtkDirectory> directory = vtkSmartPointer<vtkDirectory>::New();

  int opened = directory->Open(DirectoryName.c_str());
  if(!opened)
    {
    std::cout << "Invalid directory!" << std::endl;
    return EXIT_FAILURE;
    }

  int numFiles = directory->GetNumberOfFiles();
  std::cout << "Number of files: " << numFiles << std::endl;

  for (int i = 0; i < numFiles; i++)
    {
    std::string fileString = DirectoryName;
    fileString += "/";
    fileString += directory->GetFile(i);

    std::string ext = vtksys::SystemTools::GetFilenameLastExtension(fileString);
    std::cout << fileString << "ext: " << ext << std::endl;

    }

  return EXIT_SUCCESS;
}