#include <iostream>
// RPL is RPI Public Library
// RGRL is RPI Gereral Registration Library

#include <rgrl/rgrl_feature_based_registration.h>
#include <rgrl/rgrl_feature_set_location.h>
#include <rgrl/rgrl_initializer_prior.h>
#include <rgrl/rgrl_matcher_k_nearest.h>
#include <rgrl/rgrl_trans_translation.h>
#include <rgrl/rgrl_est_translation.h>
#include <rgrl/rgrl_convergence_on_median_error.h>

#include <rgrl/rgrl_feature_point.h>
#include <rgrl/rgrl_cast.h>
#include <rgrl/rgrl_mask.h>
#include <rgrl/rgrl_weighter_unit.h>
#include <rgrl/rgrl_converge_status.h>
#include <rgrl/rgrl_data_manager.h>
#include <rgrl/rgrl_object.h>
#include <rgrl/rgrl_event.h>
#include <rgrl/rgrl_command.h>

#include <vnl/vnl_vector.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_matrix.h>

#include <vgl/vgl_point_3d.h>

    
class command_iteration_update: public rgrl_command
{
	public:
		void execute(rgrl_object* caller, const rgrl_event & event )
		{
			execute( (const rgrl_object*) caller, event );
		}

		void execute(const rgrl_object* caller, const rgrl_event & /*event*/ )
		{
			const rgrl_feature_based_registration* reg_engine =
					dynamic_cast<const rgrl_feature_based_registration*>(caller);
			rgrl_transformation_sptr trans = reg_engine->current_transformation();
			rgrl_trans_translation* xform = rgrl_cast<rgrl_trans_translation*>(trans);
			vcl_cout<<"Xform T = "<<xform->t()<<vcl_endl;
		}
};

int main( int argc, char* argv[] )
{
	
	typedef vcl_vector< rgrl_feature_sptr >         feature_vector;
	typedef vnl_vector_fixed<double,2>              vector_2d;
	
	feature_vector  moving_feature_points;
	feature_vector  fixed_feature_points;
	
	vector_2d fixed_location;
	vector_2d moving_location;

	const unsigned int dimension = 2;
		
	for(unsigned int i = 0; i < 3; i++)
	{
		fixed_location[0] = 0;
		fixed_location[1] = i;
		fixed_feature_points.push_back( new rgrl_feature_point(fixed_location) );	
	}
	
	for(unsigned int i = 0; i < 3; i++)
	{
		moving_location[0] = 2;
		moving_location[1] = i;
		moving_feature_points.push_back( new rgrl_feature_point(moving_location) );
	}
	
	rgrl_feature_set_sptr moving_feature_set;
	rgrl_feature_set_sptr fixed_feature_set;
	moving_feature_set = new rgrl_feature_set_location<dimension>(moving_feature_points);
	fixed_feature_set =  new rgrl_feature_set_location<dimension>(fixed_feature_points);
 	
  // A translation model is used for the registration. Here we
  // instantiate an estimator for the translation transformation
	rgrl_estimator_sptr estimator = new rgrl_est_translation();
  
  // The initial transformation contains shift of 3
  // both x and y directions from the identity transform.
	
	rgrl_transformation_sptr init_transform;
	vector_2d init_parameters(3, 3);
	init_transform = new rgrl_trans_translation(init_parameters);
   
  // We now prepare the initializer for the registration.  The
  // initializer has to have the knowledge of the region of interest
  // for both the moving and fixed images, the transformation model
  // (defined by the estimator), and the initial transformation.

	vector_2d x0(0,0);          //upper left corner
	vector_2d x1(1023,1023);    //bottom right corner
	rgrl_mask_sptr moving_image_roi = new rgrl_mask_box(x0, x1);
	rgrl_mask_sptr fixed_image_roi = moving_image_roi; // assume two are identical
	rgrl_initializer_sptr initializer =
			new rgrl_initializer_prior(moving_image_roi,
			fixed_image_roi, estimator,  init_transform);
 	
  // The matcher generates the feature correspondences.
  // rgrl_matcher_k_nearest returns k nearest matches for
  // each feature. By specifying k=1, it becomes ICP

	unsigned int k = 1;
	rgrl_matcher_sptr cp_matcher = new rgrl_matcher_k_nearest( k );
	
  // The list of matches returned by the matcher may contain outliers
  // (wrong matches) due to occlusion or noise in the images. Common
  // methods to handle outliers include thresholding based on some
  // distance metric, and robust weighting, which is implemented in
  // rgrl. In this example, we use simple weighting scheme by
  // assigning a unit weight to all matches.  This results in
  // Least-Squares estimation.
	
	rgrl_weighter_sptr wgter = new rgrl_weighter_unit;
  
  // The convergence tester determines if the transformation has
  // converged and is considered good enough. The convergence test can
  // be based on the median or weighted error of the matches. In this
  // example, since we assign a unit weight to the matches, it is more
  // robust to use median error for the convergence test. The upper
  // bound of the error (tolerance) for the result to be considered
  // good enough is set to 1.5 pixels for this example. However, this
  // value has no effect on the termination of the registration
  // process. The registration process terminates only if the
  // transformation has converged, the estimation has been oscillating
  // or the number of iterations has exceeded the limit.

	double tolerance = 1.5;
	rgrl_convergence_tester_sptr conv_test = new rgrl_convergence_on_median_error( tolerance );
 	
  // We now have all the components and data ready for the
  // registration process. A registration process may contain several
  // stages, such as a number of resolutions. Each stage has its own
  // data sets, matcher, weighter, scale estimators (if needed), and
  // transformation model(s). Only the initializer and the convergence
  // tester are global to all stages.

	
	rgrl_data_manager_sptr data = new rgrl_data_manager();
	data->add_data( moving_feature_set, // data from moving image
					fixed_feature_set,  // data from fixed image
	 cp_matcher );       // matcher for this data

	rgrl_feature_based_registration reg( data, conv_test );
	
  // In the case where the estimation never converged, it is prudent
  // to establish a limit on the number of iterations, where one
  // iteration consists of matching and transformation estimation (see
  // Figure~\ref{fb-diagram}).  The default is set to 25. The number
  // can be changed using method set_max_icp_iter()
	
	reg.set_max_icp_iter(10);
	reg.set_expected_min_geometric_scale(0.001);
 	
	reg.add_observer( new rgrl_event_iteration(), new command_iteration_update());
	reg.set_debug_flag(1);
	
  // The registration process is triggered by an invocation of the method run()
	
	reg.run( initializer );
  
  // If the registration succeeds with a transformation within the
  // specified accuracy, we down-cast the transformation to the proper
  // type (rgrl_trans_translation) in order to access the
  // transformation parameters. The final alignment error is
  // accessible via the final_status() method.
	
	if ( reg.has_final_transformation() ) 
	{
		vcl_cout << "Final xform:" << vcl_endl;
		rgrl_transformation_sptr trans = reg.final_transformation();
		rgrl_trans_translation* a_xform = rgrl_cast<rgrl_trans_translation*>(trans);
		vcl_cout<< "t = "<<a_xform->t() << vcl_endl
				<< "Final alignment error = " << reg.final_status()->error() << vcl_endl;
	}
	
	return 0;
}