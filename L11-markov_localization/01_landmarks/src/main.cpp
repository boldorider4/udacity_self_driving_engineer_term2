//=================================================================================
// Name        : 1d_bayesian filter
// Version     : 1.0.0
// Copyright   : MBRDNA, Udacity
//=================================================================================

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include "measurement_package.h"
#include "map.h"
#include "help_functions.h"

using namespace std;

int main() {

  /******************************************************************************
   *  declaration:
   *****************************************************************************/

  //define example: 01, 02, 03, 04
  string example_string = "01";

  //declare map:
  map map_1d;

  //declare measurement list:
  std::vector<MeasurementPackage> measurement_pack_list;

  //declare helpers:
  help_functions helper;

  //define file names:
  char in_file_name_ctr[1024];
  char in_file_name_obs[1024];
  char in_file_name_gt[1024];

  /******************************************************************************
   *  read map and measurements:
   *****************************************************************************/
  //read map:
  helper.read_map_data("../../data/map_1d.txt", map_1d);

  //define file name of controls:
  sprintf(in_file_name_ctr, "../../data/example%s/control_data.txt",
          example_string.c_str());

  //define file name of observations:
  sprintf(in_file_name_obs, "../../data/example%s/observations/",
          example_string.c_str());

  //read in data to measurement package list:
  helper.read_measurement_data(in_file_name_ctr,
                               in_file_name_obs,
                               measurement_pack_list);

  /*****************************************************************************
   *  Coding quiz 1: just print out map infos and measurement package:     *
   ******************************************************************************/

  /////////////
  //Add code://
  /////////////
  printf("*************************************************\n");
  printf("Please print out map information!\n");
  printf("*************************************************\n");

  for (auto landmark_it : map_1d.landmark_list) {
    std::cout << "id_i " << landmark_it.id_i << " x_f " << landmark_it.x_f << std::endl;
  }

  for (auto measurement_it : measurement_pack_list) {
    if (measurement_it.observation_s_.distance_f.size() != 0)
      std::cout << "control.delta_x_f " << measurement_it.control_s_.delta_x_f << std::endl;
    else
      std::cerr << "No observations at this step!"  << std::endl
  }

  return 0;
}
