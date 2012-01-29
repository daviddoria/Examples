#ifndef THREAD_H
#define THREAD_H

#include <QThread>

class Thread : public QThread
{

public:

  virtual void run();
};

#endif