#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkLandmarkTransform.h>
#include <vtkMatrix4x4.h>

int main(int argc, char *argv[])
{
  //setup the transform
  vtkSmartPointer<vtkLandmarkTransform> landmarkTransform = 
      vtkSmartPointer<vtkLandmarkTransform>::New();
  //landmarkTransform->SetMode(VTK_LANDMARK_RIGIDBODY);
  landmarkTransform->SetMode(VTK_LANDMARK_SIMILARITY);
  //landmarkTransform->SetMode(VTK_LANDMARK_AFFINE);
  
  vtkstd::string mode = landmarkTransform->GetModeAsString();
  
  cout << "mode: " << mode << endl;
  
  return EXIT_SUCCESS;
}
