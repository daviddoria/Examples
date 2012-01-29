#ifndef POINT_H
#define POINT_H

#include <iostream>
#include <vector>

template <typename T>
void Output(std::vector<T> &V);

template <>
void Output<unsigned int>(std::vector<unsigned int> &V);

template <>
void Output<double>(std::vector<double> &V);

#include "Point.txx"

#endif

