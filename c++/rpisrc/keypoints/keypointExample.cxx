#include "keypoint.h"

#include <vector>
#include <string>
#include <iostream>

int main(int argc, char* argv[])
{
	std::string Filename = argv[1];
	std::vector< Keypoint > keypoints = ComputeKeypoints(Filename);
	
	//output
	std::cout << "Number of keypoints: " << keypoints.size() << std::endl;
  
  //for(unsigned int i = 0; i < keypoints.size(); i++)
  for(unsigned int i = 0; i < 50; i++)
    {
    Keypoint keypoint = keypoints[i];
    /*
    std::cout << "keypoint " << i << ": " << std::endl;
    std::cout << "Image coords: " << keypoint.kpt.physical_location << std::endl;
    std::cout << "Descriptor size:  " << keypoint.desc.size() << std::endl;
    std::cout << "Descriptor:  " << keypoint.desc.size() << std::endl;
    std::cout << keypoint.desc << std::endl;
    */
    }
    
	return 0;
	
}
