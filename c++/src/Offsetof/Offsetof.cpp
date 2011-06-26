#include <stdio.h>
#include <stddef.h>

struct mystruct {
  char singlechar;
  char arraymember[10];
  char anotherchar;
};

int main ()
{
  printf ("offsetof(mystruct,singlechar) is %d\n",offsetof(mystruct,singlechar));
  printf ("offsetof(mystruct,arraymember) is %d\n",offsetof(mystruct,arraymember));
  printf ("offsetof(mystruct,anotherchar) is %d\n",offsetof(mystruct,anotherchar));

  return 0;
}