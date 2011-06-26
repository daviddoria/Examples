#include <iostream>
#include <fstream>
using namespace std;

void ChangeCoutDestination();
void LogToScreen();
void LogToFile();

int main()
{
  //LogToFile();
  LogToScreen();
  return 0;
}

void ChangeCoutDestination()
{
  ofstream ofs("file.out");
  cout.rdbuf(ofs.rdbuf());
  cout << "Output." << endl;
  ofs.close();
}

void LogToScreen()
{
  clog << "Logged." << endl;
  cout << "Test" << endl;
}

void LogToFile()
{
  //save the original buffer
  streambuf *clog_save = clog.rdbuf();
  ofstream ofs("file.log");
  clog.rdbuf(ofs.rdbuf());
  clog << "Logged." << endl;
  cout << "Test" << endl;
  //reset the buffer
  clog.rdbuf(clog_save);
  ofs.close();
}