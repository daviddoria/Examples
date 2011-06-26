#include "Point.h"

template <typename T>
double Point<T>::Add()
{
    return 2.0 + 4.3;
};

// You are required to instantiate all classes you might use from this template
// since the function is not defined in the header.
template class Point<double>;
template class Point<float>;