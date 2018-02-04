#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include "vehicle.h"

using namespace std;

const double lane_width = 4.0;

const double speed_limit = 22.25;

constexpr double pi() { return M_PI; }

double deg2rad(double x);
double rad2deg(double x);

string hasData(string s);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

double getD(int lane);
int getLane(double d);

vector<Vehicle> generate_predictions(vector<vector<double>> sensor_fusion, int horizon, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

vector<Vehicle> get_vehicle_ahead(const Vehicle & vehicle, int lane, vector<Vehicle> predictions);
vector<Vehicle> get_vehicle_behind(const Vehicle & vehicle, int lane, vector<Vehicle> predictions);

vector<double> get_lane_kinematics(const Vehicle & vehicle, int lane, vector<Vehicle> predictions);

#endif

