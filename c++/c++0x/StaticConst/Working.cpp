#include <iostream>

class MyClass
{
public:
    // static const float float_sqrteps  = 3.4526698307e-4f; // this is not ok
    static const float float_sqrteps;
};

const float MyClass::float_sqrteps = 3.4526698307e-4f;

int main(int argc, char* argv[])
{
    std::cout << MyClass::float_sqrteps << std::endl;
    return 0;
}
