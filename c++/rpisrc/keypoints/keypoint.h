#ifndef KEYPOINT_H
#define KEYPOINT_H

#include <rkpl/rkpl_keypoint.h>
#include <rkpl/rkpl_scale_space.h>

#include <vnl/vnl_double_2.h>

#include <string>
#include <vector>

class Keypoint
{
  public:
  rkpl_keypoint<2>     kpt;
  vnl_vector<float>    desc;

};

std::vector< Keypoint > ComputeKeypoints(std::string Filename);
bool ComputeDescriptor( const rkpl_scale_space<float, 2> &scale_space,
                       rkpl_keypoint<2>     kpt, vnl_vector<float> & desc );

#endif