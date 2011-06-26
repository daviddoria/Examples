#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkInteractorStyleImage.h>
#include <vtkObjectFactory.h>

//#include "vtkTestFilter.h"
#include "vtkDerivedFilter.h"

class CustomStyle : public vtkInteractorStyleImage
{
  public:
    static CustomStyle* New();
    vtkTypeMacro(CustomStyle, vtkInteractorStyleImage);

    void CustomStyleCallbackFunction(vtkObject* caller,
                    long unsigned int eventId,
                    void* callData )
    {
      std::cout << "Caught event in CustomStyle" << std::endl;
    }
};
vtkStandardNewMacro(CustomStyle);

void CallbackFunction(vtkObject* caller, long unsigned int eventId, void* clientData, void* callData);

class MyClass
{
  public:
    void MyClassCallbackFunction()
    {
      std::cout << "MyClassCallbackFunction!" << std::endl;
    }
};

int main(int, char*[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();

  vtkSmartPointer<vtkCallbackCommand> progressCallback =
    vtkSmartPointer<vtkCallbackCommand>::New();
  progressCallback->SetCallback(CallbackFunction);

  /*
  vtkSmartPointer<vtkTestFilter> testFilter =
    vtkSmartPointer<vtkTestFilter>::New();
  testFilter->SetInputConnection(sphereSource->GetOutputPort());
  testFilter->AddObserver(vtkCommand::WarningEvent, progressCallback);
  testFilter->Update();
  */

  vtkSmartPointer<vtkDerivedFilter> derivedFilter =
    vtkSmartPointer<vtkDerivedFilter>::New();
  derivedFilter->SetInputConnection(sphereSource->GetOutputPort());
  derivedFilter->AddObserver(vtkCommand::WarningEvent, progressCallback);

  MyClass c;
  derivedFilter->AddObserver(vtkCommand::WarningEvent, &c, &MyClass::MyClassCallbackFunction);

  vtkSmartPointer<CustomStyle> style =
    vtkSmartPointer<CustomStyle>::New();
  derivedFilter->AddObserver(vtkCommand::WarningEvent, style, &CustomStyle::CustomStyleCallbackFunction);

  derivedFilter->Update();

  return EXIT_SUCCESS;
}

void CallbackFunction ( vtkObject* caller, long unsigned int eventId, void* clientData, void* callData )
{
  std::cout << "Called callback function!" << std::endl;
}