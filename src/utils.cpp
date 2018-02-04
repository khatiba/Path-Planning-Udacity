#include "utils.h"
#include "vehicle.h"


vector<Vehicle> get_vehicle_ahead(const Vehicle & vehicle, int lane, vector<Vehicle> predictions) {
  vector<Vehicle> vehicle_ahead;

  double min_dist = numeric_limits<double>::max();
  for (int i = 0; i < predictions.size(); i++) {
    Vehicle pred = predictions[i];
    if (pred.lane != lane || vehicle.s > pred.s) { // other lane or behind ego
      continue;
    }

    double dist = pred.s - vehicle.s;
    if (dist < min_dist) {
      min_dist = dist;
      vehicle_ahead = {pred};
    }
  }

  return vehicle_ahead;
}

vector<Vehicle> get_vehicle_behind(const Vehicle & vehicle, int lane, vector<Vehicle> predictions) {
  vector<Vehicle> vehicle_behind;

  double min_dist = numeric_limits<double>::max();
  for (int i = 0; i < predictions.size(); i++) {
    Vehicle pred = predictions[i];
    if (pred.lane != lane || vehicle.s < pred.s) { // other lane or leading ego
      continue;
    }

    double dist = vehicle.s - pred.s;
    if (dist < min_dist) {
      min_dist = dist;
      vehicle_behind = {pred};
    }
  }

  return vehicle_behind;
}

vector<double> get_lane_kinematics(const Vehicle & vehicle, int lane, vector<Vehicle> predictions) {
  double lane_speed = speed_limit;

  vector<Vehicle> vehicle_ahead = get_vehicle_ahead(vehicle, lane, predictions);
  if (vehicle_ahead.size() > 0) {
    if (vehicle_ahead[0].s - vehicle.s < 30) { // slow down closer/farther from vehicle
      lane_speed = 0.95 * vehicle_ahead[0].speed; // slightly slower than leading vehicle
    }
  }

  return {lane_speed};
}

