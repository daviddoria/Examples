#include "vtkSphericalTransform.h"
#include "vtkAbstractTransform.h"
#include "vtkMath.h"

#include <iostream>
#include <cmath>

void ManualSphereToRect(const double r, const double phi, const double theta);
void AutomaticSphereToRect(const double r, const double phi, const double theta);

void ManualRectToSphere(const double x, const double y, const double z);
void AutomaticRectToSphere(const double x, const double y, const double z);

int main(int argc, char *argv[])
{
	//double r = 1.0, phi = M_PI/3.0, theta = M_PI/4.0; //test 1
	//double r= 1.0, phi = -0.392699, theta = -0.523599;
	//ManualSphereToRect(r,phi,theta);
	//AutomaticSphereToRect(r,phi,-theta);
	
	////////////////////////////////////////
	//double x = 0.612372, y = 0.612372, z = 0.5; // inverse test 1
	
	//double x = 0.0, y = 0.0, z = 1.0;
	double x = -0.00913818, y = 0.999935, z = -0.00685384;
	  

	//ManualRectToSphere(z,y,z);
	AutomaticRectToSphere(x,y,z);
	return 0;
}

void ManualSphereToRect(const double r, const double phi, const double theta)
{
	std::cout << "ManualSphereToRect" << std::endl << " ----------- " << std::endl;
	double x = r*sin(phi)*cos(theta);
	double y = r*sin(phi)*sin(theta);
	double z = r*cos(phi);
	
	std::cout << "r = " << r << ", phi = " << phi << ", theta = " << theta << std::endl;
	std::cout << "x = " << x << ", y = " << y << ", z = " << z << std::endl;
}

void AutomaticSphereToRect(const double r, const double phi, const double theta)
{
	std::cout << "AutomaticSphereToRect" << std::endl << " ----------- " << std::endl;
	vtkSphericalTransform* SphericalTransform = vtkSphericalTransform::New();
	
	double* rect;
	double sph[3] = {r,phi,theta};;
	vtkAbstractTransform* Transform = SphericalTransform->MakeTransform ();
	
	rect = Transform->TransformPoint(sph);
	
	std::cout << "r = " << sph[0] << ", phi = " << sph[1] << ", theta = " << sph[2] << std::endl;
	std::cout << "x = " << rect[0] << ", y = " << rect[1] << ", z = " << rect[2] << std::endl;


}

///////////////////////

void ManualRectToSphere(const double x, const double y, const double z)
{
	
	//std::cout << "r = " << r << ", phi = " << phi << ", theta = " << theta << std::endl;
	//std::cout << "x = " << x << ", y = " << y << ", z = " << z << std::endl;
}

void AutomaticRectToSphere(const double x, const double y, const double z)
{
	std::cout << "AutomaticRectToSphere" << std::endl << " ----------- " << std::endl;
	
	vtkSphericalTransform* SphericalTransform = vtkSphericalTransform::New();
	double rect[3] = {x,y,z};
	double* sph;
	vtkAbstractTransform* Transform = SphericalTransform->MakeTransform()->GetInverse();
	
	sph = Transform->TransformPoint(rect);
	
	
	std::cout << "x = " << rect[0] << ", y = " << rect[1] << ", z = " << rect[2] << std::endl;
	std::cout << "r = " << sph[0] << ", phi = " << sph[1] << ", theta = " << sph[2] << std::endl;
	std::cout << "r = " << sph[0] << ", phi(deg) = " << sph[1]*180./vtkMath::Pi() << ", theta(deg) = " << sph[2]*180./vtkMath::Pi() << std::endl;


}

