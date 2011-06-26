#include <itkRGBPixel.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

void ReadFromFile(std::string &Filename);

int main(int argc, char *argv[])
{
	std::string Filename = argv[1];
	
	//float
	typedef itk::RGBPixel<float> FloatPixelType;
	FloatPixelType FP;
	FP[0]= 1.0f;
	FP[1]= 0.0f;
	FP[2]= 0.5f;
	std::cout << "FP: " << FP << std::endl;
	
	//unsigned char
	typedef itk::RGBPixel<unsigned char> UCPixelType;
	UCPixelType UCP;
	UCP[0] = 50;
	UCP[1] = 100;
	UCP[2] = 200;
	std::cout << "UCP: " << UCP << std::endl;
	
	
	ReadFromFile(Filename);
	
	//conversion
	
	return 0;
}

void ReadFromFile(std::string &Filename)
{
	std::cout << "ReadFromFile()" << std::endl << "------------" << std::endl;
	
	std::ifstream infile;
	infile.open(Filename.c_str());
	if(!infile)
	{
		std::cout << "Could not open file " << Filename << "!" << std::endl;
		return;
	}

	std::string line;
	getline(infile, line);
	std::stringstream ParsedLine(line);
	
	typedef itk::RGBPixel<unsigned char> UCPixelType;
	UCPixelType UCP;
	
	/* 
	//does not work 
	ParsedLine >> UCP[0] >> UCP[1] >> UCP[2];
	std::cout << "UCP: " << UCP << std::endl;
	*/
	
	unsigned int a,b,c;
	ParsedLine >> a >> b >> c;
	UCP[0] = static_cast<unsigned char> (a);
	UCP[1] = static_cast<unsigned char> (b);
	UCP[2] = static_cast<unsigned char> (c);
	
	std::cout << "UCP: " << UCP << std::endl;
}