#include "utils.h"
#include "vehicle.h"

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }
  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  // see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

double getD(int lane) {
  return lane_width/2 + (double)lane*lane_width;
}

int getLane(double d) {
  return (int)round((d - lane_width/2)/lane_width);
}

vector<Vehicle> generate_predictions(vector<vector<double>> sensor_fusion, int horizon, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {

  vector<Vehicle> predictions;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    double id = sensor_fusion[i][0];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double s  = sensor_fusion[i][5];
    double d  = sensor_fusion[i][6];
    double speed = sqrt(pow(vx,2) + pow(vy,2));

    double pred_s = s + (double)horizon * 0.02 * speed;
    vector<double> pred_xy = getXY(pred_s, d, maps_s, maps_x, maps_y);

    predictions.push_back(Vehicle(id, getLane(d), d, pred_s, pred_xy[0], pred_xy[1], 0, speed, "KL"));
  }

  return predictions;
}

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

vector<double> get_lane_kinematics(const Vehicle & vehicle, int lane, double look_ahead_dist, vector<Vehicle> predictions) {
  double lane_speed = speed_limit;

  vector<Vehicle> vehicle_ahead = get_vehicle_ahead(vehicle, lane, predictions);
  if (vehicle_ahead.size() > 0) {
    if (vehicle_ahead[0].s - vehicle.s < look_ahead_dist) { // slow down closer/farther from vehicle
      lane_speed = 0.95 * vehicle_ahead[0].speed; // slightly slower than leading vehicle
    }
  }

  return {lane_speed};
}

