#include <iostream>
#include <vector>
#include <cstdlib>

#include <cmath>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
		
int main(int argc, char* argv[])
{
		  
  po::options_description desc("Allowed options");
  desc.add_options()
		  ("help", "produce help message")
		  ("NumberList,N", po::value<std::vector<int> >()->multitoken(), "List of numbers.")//lets you use --NumberList or -N
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm); //assign the variables (if they were specified)
  
  if(vm.count("help")) 
  {
    std::cout << desc << std::endl;;
    return 1;
  }
	  
  if (vm.count("NumberList")) 
  {
    std::cout << "count: " << vm.count("NumberList") << std::endl;
    std::vector<int> NumberList = vm["NumberList"].as<std::vector<int> >();
    std::cout << "NumberList is length " << NumberList.size() << std::endl;
    for(unsigned int i = 0; i < NumberList.size(); i++)
    {
      std::cout << NumberList[i] << std::endl;
    }
  } 
  else 
  {
    std::cout << "NumberList was not set." << std::endl;
  }

  return 0;
}
