#ifndef MyClass_H
#define MyClass_H

#include <QObject>

#include <iostream>

class MyClass : public QObject
{
Q_OBJECT
public:
  MyClass(QObject* parent = 0){}

public slots:
  void start()
  {
    for( int count = 0; count < 5; count++ )
    {
      sleep( 1 );
      std::cout << "Ping!" << std::endl;
    }
    
    emit finished();
  }

signals:
   void finished();
};

#endif
