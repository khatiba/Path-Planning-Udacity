#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include "vehicle.h"

using namespace std;


const double speed_limit = 22.25;


vector<Vehicle> get_vehicle_ahead(const Vehicle & vehicle, int lane, vector<Vehicle> predictions);

vector<Vehicle> get_vehicle_behind(const Vehicle & vehicle, int lane, vector<Vehicle> predictions);

vector<double> get_lane_kinematics(const Vehicle & vehicle, int lane, vector<Vehicle> predictions);


#endif
