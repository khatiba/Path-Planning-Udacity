#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>


Vehicle::Vehicle(){}

Vehicle::Vehicle(double id, int lane, double d, double s, double x, double y, double yaw, double speed, string state) {
  this->id = id;
  this->lane = lane;
  this->d = d;
  this->s = s;
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->speed = speed;
  this->state = state;
}

Vehicle::~Vehicle() {}

Vehicle Vehicle::choose_next_state(vector<Vehicle> predictions) {
  vector<string> successors = successor_states();

  Vehicle best_trajectory;
  double min_cost = numeric_limits<double>::max();
  for (int i = 0; i < successors.size(); i++) {
    Vehicle trajectory = generate_trajectory(successors[i], predictions);
    /* double cost = evaluate_cost(*this, predictions, trajectory); */
    /* if (cost < min_cost) { */
    /*   min_cost = cost; */
    /*   best_trajectory = trajectory; */
    /* } */

    best_trajectory = trajectory;
    // evaluate cost of each trajectory

  }
  return best_trajectory;
}

vector<string> Vehicle::successor_states() {
  vector<string> states;
  states.push_back("KL");

  string state = this->state;
  if (state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  }
  if (state.compare("PLCL") == 0) {
    if (lane != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  }
  if (state.compare("PLCR") == 0) {
    if (lane != 2) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  //If state is "LCL" or "LCR", then just return "KL"
  return states;
}

Vehicle Vehicle::generate_trajectory(string state, vector<Vehicle> predictions) {
  Vehicle trajectory;
  /* if (state.compare("CS") == 0) { */
    /* trajectory = constant_speed_trajectory(); */
  /* } else if (state.compare("KL") == 0) { */
    trajectory = keep_lane_trajectory(predictions);
  /* } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) { */
  /*   trajectory = lane_change_trajectory(state, predictions); */
  /* } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) { */
  /*   trajectory = prep_lane_change_trajectory(state, predictions); */
  /* } */
  return trajectory;
}

/*
 * Go the speed limit or track the car in front if it's in range
 */
Vehicle Vehicle::keep_lane_trajectory(vector<Vehicle> predictions) {
  double new_speed = 49.5/2.24;

  vector<Vehicle> vehicle_ahead = get_vehicle_ahead(predictions);
  if (vehicle_ahead.size() > 0) {
    if (vehicle_ahead[0].s - this->s < 25) {
      new_speed = 0.95 * vehicle_ahead[0].speed; // may need more slowing
    }
  }

  return Vehicle(this->id, this->lane, this->d, this->s, this->x, this->y, this->yaw, new_speed, this->state);
}

/*
 * Use predictions to find any car that's in this lane ahead
 */
vector<Vehicle> Vehicle::get_vehicle_ahead(vector<Vehicle> predictions) {
  Vehicle found_vehicle;

  bool found = false;
  double min_s = numeric_limits<double>::max();
  for (int i = 0; i < predictions.size(); i++) {
    Vehicle pred = predictions[i];
    // Is there a car in my lane ahead
    if (pred.d < this->d + 2 && pred.d > this->d - 2) {
      double dist_s = (pred.s - this->s);
      if (this->s < pred.s && dist_s < min_s) {
        min_s = dist_s;
        found = true;
        found_vehicle = pred;
      }
    }
  }

  if (found) {
    return {found_vehicle};
  }

  return {};
}

