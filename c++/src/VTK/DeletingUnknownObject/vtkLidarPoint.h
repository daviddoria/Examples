#ifndef __vtkLidarPoint_h
#define __vtkLidarPoint_h

#include "vtkObject.h" //super class

class vtkRay;
//class vtkDenseArray;
#include <vtkDenseArray.h>

class vtkLidarPoint : public vtkObject
{
public:
  static vtkLidarPoint *New();
  vtkTypeRevisionMacro(vtkLidarPoint,vtkObject);

  void PrintSelf(vtkstd::ostream &os, vtkIndent indent);

  virtual void SetRay(vtkRay *);
  vtkGetObjectMacro(Ray, vtkRay);
		
protected:
  vtkLidarPoint();
  ~vtkLidarPoint();

private:
  vtkLidarPoint(const vtkLidarPoint&); //not implemented
  void operator=(const vtkLidarPoint&); //not implemented
		
  vtkRay* Ray; //the direction and location from which the point was scanned
  
  vtkDenseArray<double> Test;
  
};

#endif
