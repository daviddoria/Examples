#include <QThread>

class MyThread : public QThread
{

public:

    virtual void run();
};

void MyThread::run()
{
  for( int count = 0; count < 20; count++ )
  {
    sleep( 1 );
    qDebug( "Ping!" );
  }
}

int main()
{
  MyThread a;
  MyThread b;
  a.start();
  b.start();
  a.wait();
  b.wait();

  return 0;
}