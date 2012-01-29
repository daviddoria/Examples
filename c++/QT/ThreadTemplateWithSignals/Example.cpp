#include <ThreadTemplateWithSignals.h>
#include <iostream>

int main()
{
  ThreadTemplateWithSignals<int> a;
  a.start();
  a.wait();
  
  return 0;
}
