#include <vtkSmartPointer.h>
#include <vtkConcatenateArray.h>
#include <vtkIntArray.h>
#include <vtkDenseArray.h>

void DenseArray();
void StandardArray();

int main(int, char *[])
{
  //DenseArray();
  StandardArray();

  return EXIT_SUCCESS;
}

void DenseArray()
{
  vtkSmartPointer<vtkDenseArray<double> > array0 =
    vtkSmartPointer<vtkDenseArray<double> >::New();
  array0->Resize(2);
  array0->SetValue(0, 0);
  array0->SetValue(1, 1);

  vtkSmartPointer<vtkDenseArray<double> > array1 =
    vtkSmartPointer<vtkDenseArray<double> >::New();
  array1->Resize(2);
  array1->SetValue(0, 2);
  array1->SetValue(1, 3);

  vtkSmartPointer<vtkArrayData> arrayData0 =
    vtkSmartPointer<vtkArrayData>::New();
  arrayData0->AddArray(array0);
  arrayData0->Update();

  vtkSmartPointer<vtkArrayData> arrayData1 =
    vtkSmartPointer<vtkArrayData>::New();
  arrayData1->AddArray(array1);
  arrayData1->Update();

  vtkSmartPointer<vtkConcatenateArray> concatenateArray =
    vtkSmartPointer<vtkConcatenateArray>::New();
  concatenateArray->SetInput(0, arrayData0);
  concatenateArray->SetInput(1, arrayData1);
  concatenateArray->Update();

  vtkSmartPointer<vtkArrayData> outputArrayData = concatenateArray->GetOutput();
  vtkSmartPointer<vtkDenseArray<double> > outputArray = vtkDenseArray<double>::SafeDownCast(outputArrayData->GetArray(0));

  std::cout << outputArray->GetValue(0) << std::endl;
  std::cout << outputArray->GetValue(1) << std::endl;
  std::cout << outputArray->GetValue(2) << std::endl;
  std::cout << outputArray->GetValue(3) << std::endl;
}

void StandardArray()
{
  vtkSmartPointer<vtkIntArray> array1 =
    vtkSmartPointer<vtkIntArray>::New();
  array1->InsertNextValue(0);
  array1->InsertNextValue(1);
  array1->InsertNextValue(2);

  vtkSmartPointer<vtkIntArray> array2 =
    vtkSmartPointer<vtkIntArray>::New();
  array2->InsertNextValue(3);
  array2->InsertNextValue(4);
  array2->InsertNextValue(5);

  vtkSmartPointer<vtkArrayData> arrayData0 =
    vtkSmartPointer<vtkArrayData>::New();
  arrayData0->AddArray(array1);
  arrayData0->Update();

  /*
  vtkSmartPointer<vtkConcatenateArray> concatenateArray =
    vtkSmartPointer<vtkConcatenateArray>::New();
  concatenateArray->SetInput(0, array1);
  concatenateArray->SetInput(1, array2);
  concatenateArray->Update();
  */
}
