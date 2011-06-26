#include <vcl_iostream.h>

#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_random.h>

#include <rgrl/rgrl_feature_point.h>
#include <rgrl/rgrl_feature_set_location.h>
#include <rgrl/rgrl_trans_rigid.h>
#include <rgrl/rgrl_est_rigid.h>
#include <rgrl/rgrl_data_manager.h>
#include <rgrl/rgrl_feature_based_registration.h>

#include <rgrl/rgrl_cast.h>
#include <rgrl/rgrl_mask.h>
#include <rgrl/rgrl_converge_status.h>

typedef vnl_vector_fixed<double,2>              vector_2d;
typedef vcl_vector< rgrl_feature_sptr >         feature_vector;

// using command/observer pattern
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
			rgrl_trans_rigid* xform = rgrl_cast<rgrl_trans_rigid*>(trans);
			vcl_cout << "Xform A = " << xform->A() << "\n t= " << xform->t() << vcl_endl;
		}
};


void generate_data( feature_vector& feature_points )
{
  // Build a rectangle of size 200x100 with (50,50) at the top-left corner.
  // Gaussian noise of sigma=0.1 is added to the point set.
	//
	vnl_random random;
	double sigma  = 0.2;
	double org_x = 50;
	double org_y = 50;

  // Draw the circle, centered at (115, 115), with radius 50
	//
	double radius = 50;
	double center_x = 115;
	double center_y = 115;
	for ( unsigned int ci = 0; ci<360; ci++ ) 
	{
		vector_2d pt,tangent_dir;
		double angle = ci*2*vnl_math::pi/180;
		double next_angle = (ci+1)*2*vnl_math::pi/180;
		pt[0] = center_x + radius*vcl_cos(angle);
		pt[1] = center_y + radius*vcl_sin(angle);
		feature_points.push_back( new rgrl_feature_point(pt) );
	}
}


int	main()
{
  // Generate the feature points
	feature_vector  moving_feature_points;
	feature_vector  fixed_feature_points;

	generate_data( moving_feature_points);
	generate_data( fixed_feature_points );

  // Set up the feature sets
	const unsigned int  dimension = 2;
	rgrl_feature_set_sptr moving_feature_set =
			new rgrl_feature_set_location<dimension>(moving_feature_points);
	rgrl_feature_set_sptr fixed_feature_set =
			new rgrl_feature_set_location<dimension>(fixed_feature_points);
	const rgrl_mask_box moving_image_roi = moving_feature_set->bounding_box();
	const rgrl_mask_box fixed_image_roi = fixed_feature_set->bounding_box();

  // Set up the initial transformation and the estimator for rigid transformation

  // Next, we initialize the registration process with a prior rigid
  // transformation, and define the rigid estimator. More detailed
  // discussion on other common initialization schemes will be
  // discussed in section~\ref{sec:init}.

	vnl_matrix<double> A(2,2);
	A(0,0) = 0.996;   A(0,1) = -0.087;
	A(1,0) = -0.087;  A(1,1) = 0.996;
	vector_2d t( 10, -13 );
	rgrl_transformation_sptr init_transform = new rgrl_trans_rigid(A,t);
	rgrl_estimator_sptr      estimator = new rgrl_est_rigid(2);

  // Store the data in the data manager. Other components in the black
  // box of registration are set to the common default techniques for
  // robustness.

  // All the components required for registration are stored in a data
  // structure, \code{rgrl\_data\_manager}. This is useful for
  // illustration purposes, but not for many real problems. To make it
  // more useful, the data manager also supports robust estimation,
  // multi-resolution and multi-model registration (topics to be
  // discussed later). In the simplest form, the data manager takes
  // only the two feature sets. The data manager is passed to the
  // registration engine, \code{rgrl\_feature\_based\_registration}
  // during construction.

	rgrl_data_manager_sptr data = new rgrl_data_manager();
	data->add_data( moving_feature_set, fixed_feature_set );
	rgrl_feature_based_registration reg( data );

	reg.set_expected_min_geometric_scale( 0.1 );

  // To monitor registration
	reg.add_observer( new rgrl_event_iteration(), new command_iteration_update());

 
  // Now, we're ready to perform the registration. With the same sets
  // of features, we may obtain different results depending on the
  // region of interest (\code{image\_roi}), transformation estimator
  // ({\code{estimator}) and initial transformation estimate
  // (\code{init\_transform}). Please note, the initial estimate is
  // only for generation of the first set of matches. Therefore, it
  // can belong to a model different from that of the transformation
  // estimator.

	reg.run( moving_image_roi, fixed_image_roi, estimator, init_transform );

	if ( reg.has_final_transformation() ) {
		vcl_cout<<"Final xform:"<<vcl_endl;
		rgrl_transformation_sptr trans = reg.final_transformation();
		rgrl_trans_rigid* a_xform = rgrl_cast<rgrl_trans_rigid*>(trans);
		vcl_cout << "A = " << a_xform->A() << vcl_endl
				<< "t = " << a_xform->t() << vcl_endl
				<< "Final alignment error = " << reg.final_status()->error() << vcl_endl;
	}
	return 0;
}