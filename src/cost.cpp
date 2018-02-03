#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>


const float EFFICIENCY_WEIGHT = pow(10, 4);


double inefficiency_cost(const Vehicle & vehicle, const Vehicle & trajectory, const vector<Vehicle> & predictions) {
  double target_speed;

  if (vehicle.speed < trajectory.speed) {
    target_speed = trajectory.speed;
  } else {
    target_speed = vehicle.speed;
  }

  double cost = EFFICIENCY_WEIGHT * (2.0*target_speed - vehicle.speed - trajectory.speed)/target_speed;

  return cost;
}

float calculate_cost(const Vehicle & vehicle, const Vehicle & trajectory, vector<Vehicle> & predictions) {
  double cost = 0.0;

  cost += inefficiency_cost(vehicle, trajectory, predictions);

  return cost;
}

