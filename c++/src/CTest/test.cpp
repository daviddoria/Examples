#include <iostream>

int Add(const int a, const int b);	

int main(int argc, char *argv[])
{

	int c = Add(2, 3);

	std::cout << "2 + 3 = " << c << std::endl;

	if(c == 5)
	{
		std::cout << "Correct." << std::endl;
		return 0; //pass
	}

	std::cout << "Inorrect." << std::endl;
	return -1; //fail
}

int Add(const int a, const int b)
{
	return a + b;
}
