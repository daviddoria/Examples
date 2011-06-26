#include <iostream>
#include <iterator>
#include <vector>

int main(int argc, char *argv[])
{

	//std::back_insert_iterator<std::vector<double> > a;
	//std::back_insert_iterator<double> a;
	std::vector<double> MyVector;
	std::back_insert_iterator<std::vector<double> > MyIterator(MyVector);
	return 0;
}
