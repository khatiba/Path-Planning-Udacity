#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>


Vehicle::Vehicle(){}

Vehicle::Vehicle(double d, double s, double x, double y, double yaw, double speed, string state,
    vector<double> previous_path_x, vector<double> previous_path_y) {
  this->d = d;
  this->s = s;
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->speed = speed;
  this->state = state;
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
}

Vehicle::~Vehicle() {}

vector<string> Vehicle::successor_states() {
  vector<string> states;
  states.push_back("KL");

  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  }
  if (state.compare("PLCL") == 0) {
    if (lane != lanes_available - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  }
  if (state.compare("PLCR") == 0) {
    if (lane != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
  vector<Vehicle> trajectory;
  /* if (state.compare("CS") == 0) { */
    trajectory = constant_speed_trajectory();
  /* } else if (state.compare("KL") == 0) { */
  /*   trajectory = keep_lane_trajectory(predictions); */
  /* } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) { */
  /*   trajectory = lane_change_trajectory(state, predictions); */
  /* } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) { */
  /*   trajectory = prep_lane_change_trajectory(state, predictions); */
  /* } */
  return trajectory;
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {

}

