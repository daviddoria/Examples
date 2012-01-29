#include <itkImageFileReader.h>
#include <itkConstNeighborhoodIterator.h>

#include <rkpl/rkpl_lowe_blobs.h>
#include <rkpl/rkpl_features_for_registration.h>
#include <rkpl/rkpl_spherical_gaussian_weights.h>
#include <rkpl/rkpl_keypoint.h>

#include <vnl/vnl_double_2.h>

#include "keypoint.h"

// The factor by which to multiply the scale to get the circle radius.
const double radiusFactor = 2.0;

std::vector< Keypoint >
		ComputeKeypoints(std::string Filename)
{
  typedef float     PixelType;
  const     unsigned int    Dimension = 2;
  typedef itk::Image< PixelType, Dimension >  ImageType;

  typedef itk::ImageFileReader<ImageType> ReaderType;
  ReaderType::Pointer reader = ReaderType::New();

  double peak_height_fraction = 0.03;
  double min_peak_height = 0.1;
  reader->SetFileName(Filename.c_str());
  reader->Update();
  
  ImageType::Pointer image = reader->GetOutput();

			 
 	std::vector< Keypoint > kpts;

	try    {
    // Form scale space.
		int scales_per_octave = 4;
		int min_size = 16;
		bool rescale_intensities = true;

			rkpl_scale_space<float,2> scale_space(image,
					scales_per_octave,
     min_size,
     rescale_intensities);

      // Detect blobs.
					rkpl_lowe_blobs<float> lowe_blobs(scale_space,
							peak_height_fraction,
       min_peak_height, 5000);

					for(int j = 0; j < lowe_blobs.num_blobs(); j++)
					{
						Keypoint keypoint;
            if(lowe_blobs.get_keypoint(j, keypoint.kpt) && ComputeDescriptor(scale_space, keypoint.kpt, keypoint.desc))
						{
              kpts.push_back(keypoint);
						}
					}
		
	}
	catch(itk::ExceptionObject& e)
	{
		std::cout << e << std::endl;
	}

	return kpts;
}

bool
ComputeDescriptor( const rkpl_scale_space<float, 2> &scale_space,
                      rkpl_keypoint<2>     kpt, vnl_vector<float> & desc )
{
  
  typedef rkpl_scale_space<float,2>::image_type ImageType;
  const rkpl_keypoint<2> &k2d = kpt;

  // Find the closest available scale in the scale space.
  int smoothing_scale_index = scale_space.get_nearest_scale_index(k2d.smoothing_scale);
  
  // Get the images for the selected scale.
  const ImageType* smooth = scale_space.get_image(smoothing_scale_index,
      rkpl_scale_space<float,2>::image_L0);
  const ImageType* mag = scale_space.get_image(smoothing_scale_index,
                                               rkpl_scale_space<float,2>::image_L0_mag);
  const ImageType* ori = scale_space.get_image(smoothing_scale_index,
                                               rkpl_scale_space<float,2>::image_L0_ori);
  ImageType::RegionType region = smooth->GetLargestPossibleRegion();

  // Choose a histogram region size as a multiple of the smoothing
  // scale.  The descriptor is a 4x4 grid of histograms, each computed
  // from a square region of this size.  We want the total 4x4 region
  // to cover about 16x16 pixels in the image used for this scale.
  // Choose the constant of proportionality accordingly.

  const double m = 4;
  double rFactor = 2.0*vcl_sqrt(2.0); 
 
  double dh = m*k2d.smoothing_scale;  //sets up weighted gaussian window
  // NOTE: We might have to change this to force a 16x16 window
  // because changing the number of pixels contributing to the
  // histogram may drastically affect the results.

  // Transform the physical location to the image index and an offset.
  itk::Point<double, 2> location;
  location[0] = k2d.physical_location[0];   
  location[1] = k2d.physical_location[1];
  itk::Point<double, 2> location_truncated;
  itk::Index<2> location_index;
  smooth->TransformPhysicalPointToIndex(location, location_index);
  smooth->TransformIndexToPhysicalPoint(location_index, location_truncated);
  itk::Vector<double, 2> location_offset = location - location_truncated;

  // Compute a gaussian-weighted spherical window.
  rkpl_spherical_gaussian_weights<float, 2>
      weights(rFactor*dh, 2*dh, smooth->GetSpacing(), location_offset);

    // Make sure the region that we will use for the descriptor fits
  // inside the image.
      if(!region.IsInside(location_index - weights.GetRadius()) ||
          !region.IsInside(location_index + weights.GetRadius())) 
        return false;
 
  // Construct an iterator for the window region.
      itk::ConstNeighborhoodIterator<ImageType> 
          sit(weights.GetRadius(), smooth, smooth->GetLargestPossibleRegion());
      sit.SetLocation(location_index);
  //int s = sit.GetStride(1);
      int c = sit.GetCenterNeighborhoodIndex();

  // Construct the oriented local coordinate system based on the
  // keypoint direction.
      double orientation = vcl_atan2(k2d.direction[1], k2d.direction[0]);
      vnl_matrix_fixed<double, 2, 2> xyhat;
      xyhat(0,0) = +vcl_cos(orientation);
      xyhat(0,1) = +vcl_sin(orientation);
      xyhat(1,0) = -vcl_sin(orientation);
      xyhat(1,1) = +vcl_cos(orientation);

      itk::Point<double, 2> point;
      smooth->TransformIndexToPhysicalPoint(sit.GetIndex(c), point);

      vnl_double_2 origin;
      origin[0] = k2d.physical_location[0];
      origin[1] = k2d.physical_location[1];

  // Fill the orientation histograms.
      const int n = 8;  //number of orientation histogram bins
      desc.set_size(n*4*4);
      desc.fill(0);
      int length = weights.End() - weights.Begin();
      for(int pos = 0; pos < length; pos++)
        {
        // Compute the point location in the keypoint coordinate system
        // and scaled by the descriptor window width.
        ImageType::IndexType wah = sit.GetIndex(pos);
        smooth->TransformIndexToPhysicalPoint(sit.GetIndex(pos), point);
        vnl_double_2 p_orig = point.Get_vnl_vector()-origin;
        vnl_double_2 p = (xyhat*p_orig)/dh;

        // Get the gradient orientation in the coordinate system of the keypoint.
        float grad_ori = orientation - ori->GetPixel(sit.GetIndex(pos));
        if(grad_ori < 0) { grad_ori += (2.0*vnl_math::pi); }
        if(grad_ori >= 2.0*vnl_math::pi) { grad_ori -= (2.0*vnl_math::pi); }

        // The index of the spatial bin in which the point falls.
        int i = int(vcl_floor(p[0] + 0.5));
        int j = int(vcl_floor(p[1] + 0.5));

        // The index of the orientation bin in which the point falls.
        float t = n*(grad_ori/(2.0*vnl_math::pi));
        if(t >= 8) { t -= 8; }
        int k = int(t);

        // The fractional position within the bin.
        float dx = p[0]-(i-0.5);
        float dy = p[1]-(j-0.5);
        float dt = t - k;

        // Get the gradient magnitude and gaussian weight.
        float grad_mag = mag->GetPixel(sit.GetIndex(pos));
        float wgauss = weights.GetElement(pos);
        assert(wgauss >= 0);
        assert(grad_mag >= 0);

        // Contribute weight to each histogram center around this bin in
        // both spatial position and orientation.
        for(int idx = 0; idx < 8; ++idx)
          {
          // Weight with partial volume interpolation.
          float wx = (idx & 1)? dx : (1-dx);
          float wy = (idx >> 1 & 1)? dy : (1-dy);
          float wt = (idx >> 2 & 1)? dt : (1-dt);
          float w = wgauss*grad_mag*wx*wy*wt;
          assert(w >= 0);

          // Contribute only to bins in range.
          int ii = i+1 + (idx & 1);
          int ij = j+1 + (idx>>1 & 1);
          int ik = (k + (idx>>2 & 1))%n;
          if(ii >= 0 && ii <= 3 && ij >= 0 && ij <= 3)
            {
            desc(n*4*ij + n*ii + ik) += w;
            }
          }
        }

      // Normalize the descriptor vector.
      float two_norm = desc.two_norm();
      assert(two_norm >= 0);
      if(two_norm > 0)
        {
        desc /= two_norm;
        }
  
      // Threshold components that are too dominant and normalize again.
      double DESCRIPTOR_COMPONENT_MAXIMUM = 0.2;
      for(unsigned int i = 0; i < desc.size(); ++i)
        {
        if(desc(i) > DESCRIPTOR_COMPONENT_MAXIMUM)
          {
          desc(i) = DESCRIPTOR_COMPONENT_MAXIMUM;
          }
        }
      two_norm = desc.two_norm();
      if(two_norm > 0)
        {
        desc /= two_norm;
        }

      // Convert the descriptor to a 9-bit fixed-point representation.
      for(unsigned int i=0; i < desc.size(); ++i)
        {
        desc(i) = vcl_floor(desc(i)*512+0.5);
        }

      return true;
}