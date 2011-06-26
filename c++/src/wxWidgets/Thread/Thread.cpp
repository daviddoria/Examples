#include "wx/thread.h"
#include "wx/wx.h"

#include <iostream>

class MyThread: public wxThread
{
public:
  MyThread() : wxThread(wxTHREAD_JOINABLE){}

  void OnExit() {}
  void* Entry() {}

};

int main()
{
  wxInitialize();
  wxThread* thread = new MyThread();
  return 0;
}
