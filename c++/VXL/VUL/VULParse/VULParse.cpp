#include <iostream>
#include <string>
#include <vul/vul_arg.h>

int main(int argc, char* argv[])
{
	//setup the arguments
	
	// Create a required argument (specified by the 0) of type string. It's "help string" is "Input filename?"
	vul_arg<std::string> a_filename(0, "Input filename?");
	
	// Create a optional argument (specified by the -fast flag, instead of 0) of type bool. Its "help string" is "Input filename?". Its default value is false.
	vul_arg<bool> a_fast("-fast", "Go fast?", false);
	
	vul_arg_parse(argc, argv);
	
	std::cout << "Filename: " << a_filename() << std::endl;
	std::cout << "Go Fast?: " << a_fast() << std::endl;

  
  return 0;
}
