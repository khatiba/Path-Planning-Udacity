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
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

Vehicle Vehicle::generate_trajectory(string state, vector<Vehicle> predictions) {
  Vehicle trajectory;
  if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

Vehicle Vehicle::keep_lane_trajectory(vector<Vehicle> predictions) {
  vector<double> lane_kinematics = get_lane_kinematics(this->lane, predictions);
  double speed = lane_kinematics[0];

  return Vehicle(this->id, this->lane, this->d, this->s, this->x, this->y, this->yaw, speed, "KL");
}

Vehicle Vehicle::prep_lane_change_trajectory(string state, vector<Vehicle> predictions) {
  int next_lane = this->lane + this->lane_direction[state];

  vector<double> curr_lane_kinematics = get_lane_kinematics(this->lane, predictions);
  vector<double> next_lane_kinematics = get_lane_kinematics(next_lane, predictions);

  vector<double> best_kinematics;
  if (curr_lane_kinematics[0] < next_lane_kinematics[0]) {
    best_kinematics = curr_lane_kinematics;
  } else {
    best_kinematics = next_lane_kinematics;
  }

  double speed = best_kinematics[0];

  return Vehicle(this->id, this->lane, this->d, this->s, this->x, this->y, this->yaw, speed, state);
}

Vehicle Vehicle::lane_change_trajectory(string state, vector<Vehicle> predictions) {
  int next_lane = this->lane + this->lane_direction[state];

  for (int i = 0; i < predictions.size(); i++) {
    Vehicle pred = predictions[i];

    if (next_lane != pred.lane) { // not in the target lane
      continue;
    }

    if (this->s < pred.s + 3 && this->s > pred.s - 3) { // Car blocking the lane
      return this->keep_lane_trajectory(predictions);
    }
  }

  vector<double> next_lane_kinematics = get_lane_kinematics(next_lane, predictions);
  double speed = next_lane_kinematics[0];

  return Vehicle(this->id, next_lane, this->d, this->s, this->x, this->y, this->yaw, speed, state);
}

/*
 * Go the speed limit or track the car in front if it's in range
 */
vector<double> Vehicle::get_lane_kinematics(int lane, vector<Vehicle> predictions) {
  double speed_limit = 22; // 22m/s ~ 40mph
  double lane_speed = speed_limit; // Go the speed limit if nothing ahead

  vector<Vehicle> vehicle_ahead = get_vehicle_ahead(lane, predictions);
  if (vehicle_ahead.size() > 0) {
    if (vehicle_ahead[0].s - this->s < 20) { // slow down closer/farther from vehicle
      lane_speed = 0.95 * vehicle_ahead[0].speed; // slightly slower than leading vehicle
    }
  }

  // TODO: Check the car behind? Maybe not necessary as long as it tracks ego speed.

  return {lane_speed};
}

/*
 * Use predictions to find any car that's in this lane ahead
 */
vector<Vehicle> Vehicle::get_vehicle_ahead(int lane, vector<Vehicle> predictions) {
  vector<Vehicle> vehicle_ahead;

  double min_dist = numeric_limits<double>::max();
  for (int i = 0; i < predictions.size(); i++) {
    Vehicle pred = predictions[i];
    if (pred.lane != lane || this->s > pred.s) { // other lane or behind ego
      continue;
    }

    double dist = pred.s - this->s;
    if (dist < min_dist) {
      min_dist = dist;
      vehicle_ahead = {pred};
    }
  }

  return vehicle_ahead;
}

vector<Vehicle> Vehicle::get_vehicle_behind(int lane, vector<Vehicle> predictions) {
  vector<Vehicle> vehicle_behind;

  double min_dist = numeric_limits<double>::max();
  for (int i = 0; i < predictions.size(); i++) {
    Vehicle pred = predictions[i];
    if (pred.lane != lane || this->s < pred.s) { // other lane or leading ego
      continue;
    }

    double dist = this->s - pred.s;
    if (dist < min_dist) {
      min_dist = dist;
      vehicle_behind = {pred};
    }
  }

  return vehicle_behind;
}
