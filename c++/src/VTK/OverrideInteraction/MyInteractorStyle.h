#ifndef MYINTERACTORSTYLE_H
#define MYINTERACTORSTYLE_H


// Define interaction style
class MyInteractorStyle : public vtkInteractorStyleTrackballCamera
{
  public:
    static MyInteractorStyle* New()
    { return new MyInteractorStyle();}

    void SetActor(vtkActor* actor) { mActor = actor ; }
    void SetSource(vtkSphereSource* src) {mSrc = src;}

    virtual void OnChar() 
    {
      vtkRenderWindowInteractor *rwi = this->Interactor;		
      char ch = rwi->GetKeyCode() ;

      switch (ch) 
      {
        case '+':
        case '-':
		
          int resPhi, resTheta ;
          resPhi = mSrc->GetPhiResolution();
          resTheta = mSrc->GetThetaResolution();

          if (ch=='+') {resPhi++; resTheta++; }
          else if (ch=='-' && resPhi>0) { resPhi--; }
          else if (ch=='-' && resTheta>0) {resTheta--; }
			
          mSrc->SetPhiResolution(resPhi);
          mSrc->SetThetaResolution(resTheta);

          rwi->Render(); // render - update the screen
          break ;

        case 'c':

          double r,g,b ;

          r = vtkMath::Random(0,1);
          g = vtkMath::Random(0,1);
          b = vtkMath::Random(0,1);

          mActor->GetProperty()->SetColor(r,g,b);

          rwi->Render(); // render - update the screen
          break ;

      }
	
		// forward events
      vtkInteractorStyleTrackballCamera::OnChar();
    }
  private:
    vtkActor* mActor ;
    vtkSphereSource* mSrc ;
};

#endif