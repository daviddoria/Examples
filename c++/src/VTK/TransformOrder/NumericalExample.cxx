#include <vtkTransformPolyDataFilter.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkTransform.h>
#include <vtkXMLPolyDataWriter.h>

void Weird();
void Correct();

int main(int argc, char *argv[])
{
    Weird();
    Correct();
    return 0;
}

void Correct()
{
    vtkstd::cout << "Correct" << vtkstd::endl << "----------" << vtkstd::endl;
    vtkSmartPointer<vtkPoints> sourcePoints = vtkSmartPointer<vtkPoints>::New();

    //add a point
    double sourcePoint[3] = {0.0, 0.0, 0.0};
    sourcePoints->InsertNextPoint(sourcePoint);

    //create a polydata to add everything to
    vtkSmartPointer<vtkPolyData> originalPolydata = vtkSmartPointer<vtkPolyData>::New();
    originalPolydata->SetPoints(sourcePoints);

    /*
    //write original polydata
    vtkXMLPolyDataWriter* Writer = vtkXMLPolyDataWriter::New();
    Writer->SetFileName("orig.vtp");
    Writer->SetInput(OriginalPolydata);
    Writer->Write();
    */

    //output original points
    double origpoint[3];
    originalPolydata->GetPoint(0, origpoint);
    vtkstd::cout << "Original point: (" << origpoint[0] << ", " << origpoint[1] << ", " << origpoint[2] << ")" << vtkstd::endl;

    //create the transformation
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->PostMultiply(); //this is the key line
    transform->Translate(1.0, 0.0, 0.0);
    transform->RotateZ(90.0);

    //apply the transformation
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
        vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInput(originalPolydata);
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    vtkPolyData* transformedPolydata = transformFilter->GetOutput();

    //output transformed point
    double point[3];
    transformedPolydata->GetPoint(0, point);
    vtkstd::cout << "Transformed point: (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << vtkstd::endl;


}

void Weird()
{
    vtkstd::cout << "Weird" << vtkstd::endl << "----------" << vtkstd::endl;

    vtkSmartPointer<vtkPoints> sourcePoints = vtkSmartPointer<vtkPoints>::New();

    //add a point
    double sourcePoint[3] = {0.0, 0.0, 0.0};
    sourcePoints->InsertNextPoint(sourcePoint);

    //create a polydata to add everything to
    vtkSmartPointer<vtkPolyData> originalPolydata = vtkSmartPointer<vtkPolyData>::New();
    originalPolydata->SetPoints(sourcePoints);

    //output original points
    double origpoint[3];
    originalPolydata->GetPoint(0, origpoint);
    vtkstd::cout << "Original point: (" << origpoint[0] << ", " << origpoint[1] << ", " << origpoint[2] << ")" << vtkstd::endl;

    //create the transformation
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(1.0, 0.0, 0.0);
    transform->RotateZ(90.0);

    //apply the transformation
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
        vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInput(originalPolydata);
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    vtkPolyData* transformedPolydata = transformFilter->GetOutput();

    //output transformed point
    double point[3];
    transformedPolydata->GetPoint(0, point);
    vtkstd::cout << "Transformed point: (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << vtkstd::endl;

}

