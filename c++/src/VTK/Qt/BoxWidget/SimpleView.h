#ifndef SimpleView_H
#define SimpleView_H

#include <vtkSmartPointer.h>
#include <QMainWindow>

class vtkBoxWidget2;
class InteractorStyle;

// Forward Qt class declarations
class Ui_SimpleView;

class SimpleView : public QMainWindow
{
  Q_OBJECT
public:

  // Constructor/Destructor
  SimpleView(); 
  ~SimpleView() {}

private:
  Ui_SimpleView *ui;

  vtkSmartPointer<InteractorStyle> Style;

};

#endif // SimpleView_H
