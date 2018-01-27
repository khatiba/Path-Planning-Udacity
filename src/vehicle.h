#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

  double id;
  int lane;
  double d;
  double s;
  double x;
  double y;
  double yaw;
  double speed;
  string state;

  Vehicle();

  Vehicle(double id, int lane, double d, double s, double x, double y, double yaw, double speed, string state="KL");

  virtual ~Vehicle();

  vector<string> successor_states();
  Vehicle generate_trajectory(string state, vector<Vehicle> predictions);
  Vehicle choose_next_state(vector<Vehicle> predictions);
  vector<Vehicle> get_vehicle_ahead(vector<Vehicle> predictions);
  Vehicle keep_lane_trajectory(vector<Vehicle> predictions);
};

#endif

