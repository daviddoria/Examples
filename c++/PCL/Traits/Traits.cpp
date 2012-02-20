#include <iostream>

#include <pcl/point_types.h>
#include <pcl/for_each_type.h>

// Can't make operator() return a bool because this functor will be
// created and called for each type from the for_each_type magic,
// so the same bool reference has to be passed to each one and used.
// The idea is to set it to false initially, and if any of the fields
// match, it will get set to true.
template <typename PointT>
struct CheckPropertyXFunctor
{
  CheckPropertyXFunctor (bool &result) : result_ (result) {}

  template <typename Key> inline bool
  operator() (const std::string& fieldName)
  {
    std::string name = pcl::traits::name<PointT, Key>::value;
    if (name == fieldName)
    {
      result_ = true;
    }
  }
  bool result_;
};

int main (int argc, char** argv)
{
// Look at struct NdCopyPointEigenFunctor
  typedef pcl::PointXYZ PointT;
  typedef typename pcl::traits::fieldList<PointT>::type FieldList;

  bool result;
  pcl::for_each_type <FieldList> (CheckPropertyXFunctor <PointT> (result));
  return 0;
}

// pcl::for_each_type <FieldList> (DoesFieldExists <PointT> ("normal_x", result));
//result will be true or false
//look at NdCopyPointEigenFunctor
//imagine that but mangled with the CheckIfFieldExists
//where your coordinate values get returned too

//pcl::for_each_type <FieldList> (CopyIfFieldExists <PointT> ("normal_x", exists, value));
//and you can use value only if exists is set to true
//it might be easier to put the whole thing in the functor
//and return your VTK type directly
