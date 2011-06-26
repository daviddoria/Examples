#include "itkBinaryNotImageFilter.h"
#include "itkCommand.h"

typedef itk::Image<unsigned char, 2>  ImageType;

class MyCommand : public itk::Command
{
  public:
    itkNewMacro( MyCommand );

  public:

    void Execute(itk::Object *caller, const itk::EventObject & event)
    {
      Execute( (const itk::Object *)caller, event);
    }

    void Execute(const itk::Object * object, const itk::EventObject & event)
    {
      std::cout << "Command called." << std::endl;
    }

};

int main(int, char*[])
{
  ImageType::Pointer image = ImageType::New();
  typedef itk::BinaryNotImageFilter <ImageType>
          BinaryNotImageFilterType;

  BinaryNotImageFilterType::Pointer filter = BinaryNotImageFilterType::New();
  filter->SetInput(image);

  MyCommand::Pointer myCommand = MyCommand::New();
  filter->AddObserver(itk::ProgressEvent(), myCommand);

  filter->Update();


  return EXIT_SUCCESS;
}
