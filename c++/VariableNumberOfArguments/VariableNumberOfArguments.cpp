#include <iostream>
#include <stdarg.h>

void Function1(unsigned int num, ...);
void Function2(unsigned int num, va_list ap);

int main(int argc, char *argv[])
{
  Function1(3, 1.0, 2.0, 3.0);
  return 0;
}

void Function1(unsigned int num, ...)
{
  va_list ap;

  va_start(ap, num);

  Function2(num, ap);

}

void Function2(unsigned int num, va_list ap)
{
  for (unsigned int i = 0; i < num; i++)
    {
    double val = va_arg(ap,double);
    //printf ("\t%.2f",val);
    std::cout << "Val " << i << " : " << val << std::endl;
    }
    
  va_end(ap);

}
