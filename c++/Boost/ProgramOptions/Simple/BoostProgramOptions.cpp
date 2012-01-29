#include <iostream>
#include <vector>
#include <cstdlib>

#include <cmath>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
		
int main(int argc, char* argv[])
{
  int compression;
  std::vector<std::string> FileList;

  std::vector<std::vector<std::string> > Lists(2);
		  
  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("compression", po::value<int>(&compression)->default_value(5), "set compression level") //automatically assign to a variable

      ("size", po::value<double>()->default_value(10), "set size") //use a default value
      ("FileList", po::value<std::vector<std::string> >(&FileList)->multitoken(), "List of files.") //multiple values
      ("NumberList,N", po::value<std::vector<int> >()->multitoken(), "List of numbers.")//lets you use --NumberList or -N
      ("List0", po::value<std::vector<std::string> >(&Lists[0])->multitoken(), "List0.")
      ("List1", po::value<std::vector<std::string> >(&Lists[1])->multitoken(), "List1.")
      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm); //assign the variables (if they were specified)
  
  std::cout << "compression: " << compression << std::endl;

  if(vm.count("help")) 
  {
    std::cout << desc << std::endl;;
    return 1;
  }
  
  if(vm.count("compression")) 
  {
    std::cout << "Compression level was set to "
	      << vm["compression"].as<int>() << std::endl;
  } 
  else 
  {
    std::cout << "Compression level was not set." << std::endl;
  }
  

  if (vm.count("size")) 
  {
    std::cout << "Size was set to "
	      << vm["size"].as<double>() << std::endl;
  } 
  else 
  {
    std::cout << "Size was not set." << std::endl;
  }

  if (vm.count("FileList")) 
  {
    std::cout << "count: " << vm.count("FileList") << std::endl;
    //vector<string> FileList = vm["FileList"].as<vector<string> >();
    std::cout << "FileList is length " << FileList.size() << std::endl;
    for(unsigned int i = 0; i < FileList.size(); i++)
    {
      std::cout << FileList[i] << std::endl;
    }
  } 
  else 
  {
    std::cout << "FileList was not set." << std::endl;
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


  for(unsigned int list = 0; list < Lists.size(); list++)
  {
    std::cout << "List: " << list << std::endl;
    for(unsigned int item = 0; item < Lists[list].size(); item++)
    {
	    std::cout << Lists[list][item] << std::endl;
    }
  }
  
  return 0;
}
