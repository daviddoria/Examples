#include <iostream>
#include <vector>
#include <cstdlib>

#include <cmath>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

using namespace std;

int main(int argc, char* argv[])
{
	vector<vector<string> > lists;
			
	po::options_description desc("Allowed options");
	desc.add_options()
			("help", "produce help message")
			("list", po::value<vector<string> >()->multitoken(), "Lists.")
			;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm); //assign the variables (if they were specified)

	if (vm.count("help"))
	{
		cout << desc << "\n";
		return 1;
	}

	cout << "There are " << vm.count("list") << " lists." << endl;
	
	for(unsigned int list = 0; list < vm.count("list"); list++)
	{
		vector<string> l = vm["list"].as<vector<string> >();
		lists.push_back(l);
		
		cout << "List: " << list << endl << "-----------" << endl;
		for(unsigned int item = 0; item < lists[list].size(); item++)
		{
			cout << "List: " << list << " Item: " << item << " " << lists[list][item] << endl;
		}
		
	}
	
	return 0;
}
