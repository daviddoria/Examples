#ifndef ThreadTemplateWithSignals_H
#define ThreadTemplateWithSignals_H

#include <QThread>
#include <iostream>

class NonTemplateThread : public QThread
{
Q_OBJECT

signals:
  void TestSignal();
};


template <typename TFilter>
class ThreadTemplateWithSignals : public NonTemplateThread
{
public:
  void run();

};

#include "ThreadTemplateWithSignals.hxx"

#endif
