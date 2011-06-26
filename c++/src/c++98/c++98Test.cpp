#include <iostream>
#include <vector>
#include <algorithm>

int main()
{
	//vector initialization
	//std::vector<int> v = {1,2,3};
		
	std::vector<int> v;
	v.push_back(1);
	v.push_back(2);
	int m = min_element(v.begin(), v.end());
	std::cout << m << "\n";
	return 0;
}
