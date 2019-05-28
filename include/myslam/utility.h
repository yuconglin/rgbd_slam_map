#ifndef UTILITY_H
#define UTILITY_H

#include "myslam/common_include.h"

namespace myslam
{
namespace utility 
{
template <typename T>
const T &GetMedian(vector<T> &len)
{
  assert(!len.empty());
  if (len.size() % 2 == 0)
  {
    const auto median_it1 = len.begin() + len.size() / 2 - 1;
    const auto median_it2 = len.begin() + len.size() / 2;

    std::nth_element(len.begin(), median_it1, len.end());
    const auto e1 = *median_it1;

    std::nth_element(len.begin(), median_it2, len.end());
    const auto e2 = *median_it2;

    return (e1 + e2) / 2;
  }
  else
  {
    const auto median_it = len.begin() + len.size() / 2;
    std::nth_element(len.begin(), median_it, len.end());
    return *median_it;
  }
};
} // namespace utility
} // namespace myslam

#endif