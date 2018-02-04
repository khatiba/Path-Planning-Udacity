#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "utils.h"

using namespace std;

using json = nlohmann::json;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  double ref_speed = 0;
  double target_speed = 0;
  int target_lane = 1;
  string state = "KL";

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &ref_speed, &target_lane, &target_speed, &state]
      (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          car_speed /= 2.24;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          // Create the spline with an initial tangent path for smooth transition
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          Vehicle ego = Vehicle(0, target_lane, car_d, car_s, ref_x, ref_y, ref_yaw, car_speed, state);
          vector<Vehicle> predictions = generate_predictions(sensor_fusion, prev_size, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ego = ego.choose_next_state(predictions);

          target_speed = ego.speed;
          target_lane = ego.lane;
          state = ego.state;

          vector<double> ptsx;
          vector<double> ptsy;

          if (prev_size < 2) {
            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
            ptsx.push_back(car_x + cos(ref_yaw));
            ptsy.push_back(car_y + sin(ref_yaw));
          } else {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];

            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsy.push_back(ref_y_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);
          }

          for (int next_s = 30; next_s <= 90; next_s += 30) {
            vector<double> wp = getXY(car_s + next_s, getD(target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(wp[0]);
            ptsy.push_back(wp[1]);
          }

          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          tk::spline s;
          s.set_points(ptsx, ptsy);

          // start with all remaining points from previous path
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          if (ref_speed < target_speed)
            ref_speed += 0.1;
          if (ref_speed > target_speed)
            ref_speed -= 0.1;
          if (ref_speed > speed_limit)
            ref_speed = speed_limit;

          double target_x = 50;
          double target_y = s(target_x);
          double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
          double N = target_dist/(0.02*ref_speed);
          double distance_inc = target_x/N;

          for (int i = 0; i < 50 - prev_size; i++) {
            double next_x = distance_inc * (i+1);
            double next_y = s(next_x);

            double map_next_x = ref_x + next_x * cos(ref_yaw) - next_y * sin(ref_yaw);
            double map_next_y = ref_y + next_x * sin(ref_yaw) + next_y * cos(ref_yaw);

            next_x_vals.push_back(map_next_x);
            next_y_vals.push_back(map_next_y);
          }

          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

