#ifndef __vtkRay_h
#define __vtkRay_h

template <typename T> class vtkSmartPointer;
//#include "vtkSmartPointer.h" //it works if you use this

class vtkTransform;

#include "vtkObject.h" //superclass

class vtkRay : public vtkObject
{
	public:
		static vtkRay *New();
    vtkTypeRevisionMacro(vtkRay,vtkObject);
		void PrintSelf(ostream &os, vtkIndent indent);

	protected:
		vtkRay();
		~vtkRay();
	private:
		vtkRay(const vtkRay&);  // Not implemented.
		void operator=(const vtkRay&);  // Not implemented.
    
    vtkSmartPointer<vtkTransform> Transform;
};

#endif