#ifndef __vtkRay_h
#define __vtkRay_h

#include <iostream>

class vtkTransform;
#include "vtkObject.h" //superclass

/*
This class is a container for a point and a direction - a very commonly used collection of objects.
*/
				 
class vtkRay : public vtkObject
{
	public:
		static vtkRay *New();
    vtkTypeRevisionMacro(vtkRay,vtkObject);
		void PrintSelf(ostream &os, vtkIndent indent);
		
		////////// Accessors ///////////
		vtkGetVector3Macro(Origin,double);
		vtkGetVector3Macro(Direction,double);
		
		/////////// Mutators ///////////
		vtkSetVector3Macro(Origin,double); //specify the origin of the ray
		
		void SetDirection(double* Dir); //specify the direction of the ray
    
    //operator vtkVariant() const { return vtkVariant(); } //for vtkDenseArray compatibility
    
	protected:
		vtkRay();
		~vtkRay();
	private:
		vtkRay(const vtkRay&);  // Not implemented.
		void operator=(const vtkRay&);  // Not implemented.
    
    double Origin[3];//(x,y,z) coords of ray origin
    double Direction[3];//the direction of the ray
};

#endif