//============================================================================
// Name        : bayesianFilter.cpp
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//============================================================================

#include "bayesianFilter.h"
#include <iostream>
#include <algorithm>
#include <vector>

using namespace std;

//constructor:
bayesianFilter::bayesianFilter() : is_initialized_(false), control_std_(1.0f) {
	bel_x.resize(100,0);
	bel_x_init_ = bel_x;
}

//de-constructor:
bayesianFilter::~bayesianFilter() {

}

void bayesianFilter::process_measurement(const MeasurementPackage &measurements,
                                         const map &map_1d,
                                         help_functions &helpers){

	/******************************************************************************
	 *  Set init belief of state vector:
	 ******************************************************************************/
  if (!is_initialized_) {

		//run over map, all map_1d.lanmark_list values:
    for (auto landmark_it = map_1d.landmark_list.begin(); landmark_it != map_1d.landmark_list.end();
         ++landmark_it) {

			//get landmark l from map
			//check, if landmark position x fits in map [0,100]:
      if(landmark_it->x_f > 0 || landmark_it->x_f < bel_x_init_.size()) {

				//get landmark x position * use help_function.h for reference

				//cast float to int:
				int position_x = static_cast<int>(landmark_it->x_f) ;
				//set belief to 1:
				bel_x_init_[position_x]   = 1.0f;
				bel_x_init_[position_x-1] = 1.0f;
				bel_x_init_[position_x+1] = 1.0f;
      }

    }


    //normalize belief at time 0:
    bel_x_init_ = helpers.normalize_vector(bel_x_init_);

    //set initial flag to true:
    is_initialized_ = true;

  }

	/******************************************************************************
	 *  motion model and observation update
	******************************************************************************/
	std::cout <<"-->motion model for state x ! \n" << std::endl;

	//get current observations and control information:
	MeasurementPackage::control_s     controls = measurements.control_s_;
	MeasurementPackage::observation_s observations = measurements.observation_s_;

	//run over all bel_x values (index represents the pose in x!):
	//for (int i=0; i< ...){


		/**************************************************************************
		 *  posterior for motion model
		**************************************************************************/

        // motion posterior:
        // used to set up the convlution
		float posterior_motion = 0.0f;

		//loop over state space x_t-1 * same size as bel_x (Perform Convolution):
		//for (int j=0; j< ...){

			//TODO: Calculate transition probabilites using helpers.normpdf()
			// x: difference between bel_x index and state space index
			// mu: the movement from controls defined above
			// std: defined eariler

			//TODO: Calculate motion model
			// ADD the transition prob multiplied by the intial believe
			// at state space index
			//posterior_motion +=
		//}

		//TODO: update = motion_model
		// set as the posterior_motion
		//bel_x[i] =


	//};
		//TODO: normalize bel_x:


		//TODO: set initial believe to bel_x for next time
		//bel_x_init =




};
