#ifndef __vtkLidarPoint_h
#define __vtkLidarPoint_h

#include "vtkObject.h" //super class
#include "vtkRay.h"

#include "vtkSmartPointer.h"

class vtkLidarPoint : public vtkObject
{
	public:
    static vtkLidarPoint *New();
    void PrintSelf(vtkstd::ostream &os, vtkIndent indent);

    vtkGetObjectMacro(Ray, vtkRay);
    vtkSetObjectMacro(Ray, vtkRay);
		
	protected:

		vtkLidarPoint();
		~vtkLidarPoint();
	private:
		vtkLidarPoint(const vtkLidarPoint&); //not implemented
		void operator=(const vtkLidarPoint&); //not implemented
		
		vtkRay* Ray; //the direction and location from which the point was scanned
    //vtkSmartPointer<vtkRay> Ray;
		
};

#endif