#include <vcl_iostream.h>

#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_random.h>

#include <rgrl/rgrl_feature_point.h>
#include <rgrl/rgrl_feature_set_location.h>

#include <rgrl/rgrl_data_manager.h>
#include <rgrl/rgrl_feature_based_registration.h>


#include <rgrl/rgrl_trans_rigid.h>
#include <rgrl/rgrl_est_rigid.h>

#include <rgrl/rgrl_cast.h>
#include <rgrl/rgrl_mask.h>
#include <rgrl/rgrl_converge_status.h>
#include <rgrl/rgrl_matcher_k_nearest.h>

typedef vnl_vector_fixed<double,2>              vector_2d;
typedef vnl_vector_fixed<double,3>              vector_3d;
typedef vcl_vector< rgrl_feature_sptr >         feature_vector;

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
			vcl_cout<<"Xform R = "<<xform->R()<<"\n t= "<<xform->t()<<vcl_endl;
		}
};


int	main()
{
	const unsigned int  dimension = 3;
	
	vector_3d pt;
			
	//create moving features
	feature_vector  moving_feature_points;
	
	double eps = 0.0;
	
	pt[0] = 0.1;
	pt[1] = 0.0;
	pt[2] = 0.0;
	moving_feature_points.push_back( new rgrl_feature_point(pt) );
	
	pt[0] = 0.0;
	pt[1] = 1.0;
	pt[2] = 0.0;
	moving_feature_points.push_back( new rgrl_feature_point(pt) );
	
	pt[0] = 0.0;
	pt[1] = 2.0;
	pt[2] = 0.0;
	moving_feature_points.push_back( new rgrl_feature_point(pt) );
	
    
    
	//create fixed features
	feature_vector  fixed_feature_points;
	
	pt[0] = 1.0 - eps;
	pt[1] = 0.0;
	pt[2] = 0.0;
	fixed_feature_points.push_back( new rgrl_feature_point(pt) );
	
	pt[0] = 1.1 - eps;
	pt[1] = 1.0;
	pt[2] = 0.0;
	fixed_feature_points.push_back( new rgrl_feature_point(pt) );
	
	pt[0] = 0.9 - eps;
	pt[1] = 2.0;
	pt[2] = 0.0;
	fixed_feature_points.push_back( new rgrl_feature_point(pt) );
		
	//construct feature sets
	bool UseBins = true;
	rgrl_feature_set_sptr moving_feature_set = new rgrl_feature_set_location<dimension>(moving_feature_points); // false = do not use "bins"
	rgrl_feature_set_sptr fixed_feature_set = new rgrl_feature_set_location<dimension>(fixed_feature_points);// false = do not use "bins"
	
    std::cout << "Moving features BB: " << moving_feature_set->bounding_box() << std::endl;
    
	rgrl_matcher_k_nearest * mkn = new rgrl_matcher_k_nearest(1);
	mkn->set_debug_flag(10);
	
	//const rgrl_mask_box moving_image_roi = moving_feature_set->bounding_box();
    const rgrl_mask_box moving_image_roi(vnl_vector_fixed<double,3>(0.0, 0.0, 0.0), vnl_vector_fixed<double,3>(2.0, 2.0, 2.0));
    
	//const rgrl_mask_box fixed_image_roi = fixed_feature_set->bounding_box();
    const rgrl_mask_box fixed_image_roi(vnl_vector_fixed<double,3>(0.0, 0.0, 0.0), vnl_vector_fixed<double,3>(2.0, 2.0, 2.0));
 	
	//construct initial transformation
	vnl_matrix<double> R(3,3);
	R(0,0) = 1.0;   R(0,1) = 0.0; R(0,2) = 0.0;
	R(1,0) = 0.0;  R(1,1) = 1.0; R(1,2) = 0.0;
	R(2,0) = 0.0;  R(2,1) = 0.0; R(2,2) = 1.0;
	vector_3d t( 0.0, 0.0, 0.0 );
	rgrl_transformation_sptr init_transform = new rgrl_trans_rigid(R, t);
	rgrl_estimator_sptr estimator = new rgrl_est_rigid(3);

	//data manager?
	rgrl_data_manager_sptr data = new rgrl_data_manager();
	//data->set_debug_flag(1);
	data->add_data( moving_feature_set, fixed_feature_set, mkn );
	std::cout << data->debug_flag()<< std::endl;
	rgrl_feature_based_registration reg( data );
	//reg.set_debug_flag(10);

	//reg.set_expected_min_geometric_scale( 0.1 );

  // To monitor registration
	//reg.add_observer( new rgrl_event_iteration(), new command_iteration_update());
	
	reg.run( moving_image_roi, fixed_image_roi, estimator, init_transform );

	if ( reg.has_final_transformation() ) 
	{
		vcl_cout<<"Final xform:"<<vcl_endl;
		rgrl_transformation_sptr trans = reg.final_transformation();
		rgrl_trans_rigid* a_xform = rgrl_cast<rgrl_trans_rigid*> (trans);
		vcl_cout << "R = " << a_xform->R() << vcl_endl
				<< "t = " << a_xform->t() << vcl_endl
				<< "Final alignment error = " << reg.final_status()->error() << vcl_endl;
	}

}