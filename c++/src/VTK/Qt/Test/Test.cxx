#include <QApplication>
#include <QVTKWidget.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkBorderWidget.h>

int main(int argc, char** argv)
{
  QApplication app(argc, argv);
  QVTKWidget w;
  w.resize(400,400);
  vtkSmartPointer<vtkRenderer> ren = 
      vtkSmartPointer<vtkRenderer>::New();
  w.GetRenderWindow()->AddRenderer(ren);
  vtkSmartPointer<vtkBorderWidget> border = 
      vtkSmartPointer<vtkBorderWidget>::New();
  border->SetInteractor(w.GetInteractor());
  border->On();
  w.show();
  return app.exec();
}

#if 0
#include <QApplication>
#include <QCleanlooksStyle>
#include "SimpleView.h"

extern int qInitResources_icons();

int main( int argc, char** argv )
{
  
  // QT Stuff
  QApplication app( argc, argv );

  QApplication::setStyle(new QCleanlooksStyle);
  
  qInitResources_icons();
  
  SimpleView mySimpleView;
  mySimpleView.show();
  
  return app.exec();
}
#endif