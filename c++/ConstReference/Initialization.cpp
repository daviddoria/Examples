
class MyClass
{
 public:
 //MyClass(const int& a) : A(a){}  // This is identical to the next line
 MyClass(int const& a) : A(a){}
 int const& A;
};
 
int main()
{
 int b = 4;
 MyClass myClass(b);
 return 0;
}