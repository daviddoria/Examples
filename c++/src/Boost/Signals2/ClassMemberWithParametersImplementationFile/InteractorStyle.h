#ifndef InteractorStyle_H
#define InteractorStyle_H

#include <boost/signals2/signal.hpp>

class InteractorStyle
{
public:
  boost::signals2::signal<void (float)> UpdateData;
};

#endif