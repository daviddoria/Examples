#include <iostream>
#include <vector>

#include <boost/thread/thread.hpp>

class CSharedData
{
public:
    CSharedData() : data(0) { }

    void send (int m)
    {
      data = m;
    }
    
    int receive()
    {
      return data;
    }

private:
    int data;

};

CSharedData SharedData;

void sender()
{
  SharedData.send(21);
  int data = SharedData.receive();
  std::cout << "Data in sender: " << data << std::endl;
}

void receiver()
{
  int n = 0;
  while (n < 100000000)
  {
    ++n;
  }

  int data = SharedData.receive();
  std::cout << "Data in receiver: " << data << std::endl;
}

int main(int, char*[])
{
  boost::thread thrd1(&sender);
  boost::thread thrd2(&receiver);

  thrd1.join();
  thrd2.join();

  return 0;
}
