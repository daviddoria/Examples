#include <iostream>

#include <pcl/point_types.h>
#include <pcl/for_each_type.h>

template <typename PointT>
struct CheckPropertyXFunctor
{
  CheckPropertyXFunctor (bool &result) : result_ (result) {}

  template <typename Key> inline void
  operator() ()
  {
    std::string name = pcl::traits::name<PointT, Key>::value;
    if (name == "normal_x")
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

//set result_ to false in the constructor maybe or something
//you get the drift :)
//you can also pass in the type of "property"
//in the constructor as a string
//and make this a general CheckIfFieldExists sort of a functor


// pcl::for_each_type <FieldList> (DoesFieldExists <PointT> ("normal_x", result));
//result will be true or false
//look at NdCopyPointEigenFunctor
//imagine that but mangled with the CheckIfFieldExists
//where your coordinate values get returned too

//pcl::for_each_type <FieldList> (CopyIfFieldExists <PointT> ("normal_x", exists, value));
//and you can use value only if exists is set to true
//it might be easier to put the whole thing in the functor
//and return your VTK type directly
