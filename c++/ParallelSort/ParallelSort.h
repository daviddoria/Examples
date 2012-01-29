#ifndef PARALLELSORT_H
#define PARALLELSORT_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

namespace ParallelSort
{
  
template <typename T>
void OutputIndexedValueVector(const std::vector<T>& v);

template <typename T>
void OutputVector(const std::vector<T>& v);

template <typename T>
struct IndexedValue
{
  unsigned int index;
  T value;
};

template <typename T>
bool operator<(IndexedValue<T> element1, IndexedValue<T> element2);

template <typename T>
std::vector<IndexedValue<T> > ParallelSort(const std::vector<T>& v);

} // end namespace

#include "ParallelSort.hxx"

#endif
