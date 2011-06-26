#define SetMacro(name,type) \
void Set##name (type _arg)

// run
// gcc -E file.cpp
// to see the precompiler output
int main()
{
  SetMacro(Test, int);
}
