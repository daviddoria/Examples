#ifndef EASYRENDERER_H
#define EASYRENDERER_H

#include "vtkSmartPointer.h"
#include "vtkPolyData.h"

class EasyRenderer
{
  public:
  void Render();
  void AddObject(vtkSmartPointer<vtkPolyData> Object);
  
  private:
  vtkstd::vector<vtkSmartPointer<vtkPolyData> > Objects;
};



#endif