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

  double d;
  double s;
  double x;
  double y;
  double yaw;
  double speed;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  string state;

  Vehicle();

  Vehicle(double d, double s, double x, double y, double yaw, double speed, string state="CS",
      vector<double> previous_path_x, vector<double> previous_path_y);

  virtual ~Vehicle();

  vector<Vehicle> constant_speed_trajectory();
};

#endif

