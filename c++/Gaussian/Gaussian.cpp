#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>

double GaussianAmplitude(double mean, double variance, double distanceFromMean)
		
int main(int argc, char *argv[])
{
  GaussianAmplitude(0, 1, 0);
  
  return 0;
}

double GaussianAmplitude(double mean, double variance, double distanceFromMean)
{
  return 1./(sqrt(2.*vtkMath::Pi() * variance)) * exp(-(pow(distanceFromMean,2))/(2.*variance));
}
