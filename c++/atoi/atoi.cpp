#include <cstdio>
#include <cstdlib>

int main ()
{
  int i;
  char szInput [256];
  printf ("Enter a number: ");
  fgets ( szInput, 256, stdin );
  i = atoi (szInput);
  printf ("The value entered is %d. The double is %d.\n",i,i*2);
  return 0;
}