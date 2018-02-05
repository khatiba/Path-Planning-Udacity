#include "cost.h"
#include "vehicle.h"
#include "utils.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>


const double EFFICIENCY_WEIGHT = pow(10, 4);


double calculate_cost(const Vehicle & vehicle, const Vehicle & trajectory, const vector<Vehicle> & predictions) {
  double cost = 0.0;

  cost += inefficiency_cost(vehicle, trajectory, predictions);

  return cost;
}

double inefficiency_cost(const Vehicle & vehicle, const Vehicle & trajectory, const vector<Vehicle> & predictions) {

  map<string, int> lane_direction = {{"PLCL", -1}, {"PLCR", 1}};

  int final_lane = trajectory.lane;
  int intended_lane = trajectory.lane;
  if (lane_direction.find(trajectory.state) != lane_direction.end() ) {
    intended_lane = intended_lane + lane_direction[trajectory.state];
  }

  vector<double> intended_kinematics = get_lane_kinematics(vehicle, intended_lane, 40.0, predictions);
  vector<double> final_kinematics = get_lane_kinematics(vehicle, final_lane, 40.0, predictions);

  double intended_speed = intended_kinematics[0];
  double final_speed = final_kinematics[0];

  // Don't jump between lanes if it's just slightly faster.
  if (abs(intended_speed - final_speed) < 1.5) {
    final_speed = intended_speed;
  }

  double cost = EFFICIENCY_WEIGHT * (2.0*speed_limit - intended_speed - final_speed)/speed_limit;

  return cost;
}

