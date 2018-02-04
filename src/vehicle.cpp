#include <algorithm>
#include <iostream>
#include "cost.h"
#include "utils.h"
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
    string state = successors[i];

    Vehicle trajectory = generate_trajectory(state, predictions);

    double cost = calculate_cost(*this, trajectory, predictions);

    cout << state << ": " << round(cost) << endl;

    /* cout << setw(5) << successors[i] << round(cost) << endl; */
    if (cost < min_cost) {
      min_cost = cost;
      best_trajectory = trajectory;
    }
  }

  return best_trajectory;
}

vector<string> Vehicle::successor_states() {
  vector<string> states;
  states.push_back("KL");

  string state = this->state;
  if (state.compare("KL") == 0) {
    if (this->lane > 0) {
      states.push_back("PLCL");
    }
    if (this->lane < 2) {
      states.push_back("PLCR");
    }
  }
  if (state.compare("PLCL") == 0) {
    if (this->lane > 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  }
  if (state.compare("PLCR") == 0) {
    if (this->lane < 2) {
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
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  }
  return trajectory;
}

Vehicle Vehicle::keep_lane_trajectory(vector<Vehicle> predictions) {
  vector<double> lane_kinematics = get_lane_kinematics(*this, this->lane, predictions);
  double speed = lane_kinematics[0];

  return Vehicle(this->id, this->lane, this->d, this->s, this->x, this->y, this->yaw, speed, "KL");
}

Vehicle Vehicle::prep_lane_change_trajectory(string state, vector<Vehicle> predictions) {
  int next_lane = this->lane + this->lane_direction[state];

  vector<double> curr_lane_kinematics = get_lane_kinematics(*this, this->lane, predictions);
  vector<double> next_lane_kinematics = get_lane_kinematics(*this, next_lane, predictions);

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

    if (this->s < pred.s + 3 && this->s > pred.s - 35) { // Car blocking the lane
      cout << "Can't Change Lanes: " << state << endl;
      return this->keep_lane_trajectory(predictions);
    }
  }

  vector<double> next_lane_kinematics = get_lane_kinematics(*this, next_lane, predictions);
  double speed = next_lane_kinematics[0];

  return Vehicle(this->id, next_lane, this->d, this->s, this->x, this->y, this->yaw, speed, state);
}

