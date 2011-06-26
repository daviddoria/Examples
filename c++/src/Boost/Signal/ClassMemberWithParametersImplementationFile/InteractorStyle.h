#ifndef InteractorStyle_H
#define InteractorStyle_H

#include <boost/signal.hpp>

class InteractorStyle
{
public:
  boost::signal<void (float)> UpdateData;
};

#endif