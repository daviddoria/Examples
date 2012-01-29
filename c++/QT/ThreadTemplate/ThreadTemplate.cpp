#include <QThread>
#include <iostream>

template <typename T>
class MyThread : public QThread
{
public:
  void run();
  void SetValue(T value){this->Value = value;}
  void OutputValue(){std::cout << this->Value << std::endl;}
  T Value;
};

template <typename T>
void MyThread<T>::run()
{
  for( int count = 0; count < 20; count++ )
    {
    sleep( 1 );
    //qDebug( "Ping!" );
    this->OutputValue();
    }
}

int main()
{
  MyThread<int> a;
  a.SetValue(1.2);
  MyThread<double> b;
  b.SetValue(1.2);
  a.start();
  b.start();
  a.wait();
  b.wait();

  return 0;
}