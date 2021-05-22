#include <uWS/uWS.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
#define toM_S 0.44704

Eigen::Vector2d translation(const vector<double> &trans,
                            Eigen::Vector2d input) {
  Eigen::Vector2d trans_vector(trans.data());
  assert(trans.size() == input.size());
  return trans_vector + input;
}

Eigen::Vector2d rotation2d(Eigen::Vector2d input, double theta) {
  Eigen::Matrix2d rotation;
  rotation << cos(theta), -sin(theta), sin(theta), cos(theta);
  return rotation * input;
}

void printVector(const string msg, const vector<double> &v) {
  std::cout << msg;
  for (unsigned int i = 0; i < v.size(); ++i) {
    std::cout << std::setprecision(10) << v[i] << "\t";
  }
  std::cout << std::endl;
}

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
  // variables passed to onMessage()
  Vehicle ego;
  const int num_lanes = 3;
  const int lane_width = 4;
  // calculate 50 MPH to m/s and reduce 1%
  const float speed_limit = 50 * 0.99;  // MPH
  const float accel_limit = 10 * 0.5;   // m/s^2
  ego.configure(num_lanes, lane_width, speed_limit * toM_S, accel_limit, max_s);

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&ego, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s,
               &map_waypoints_dx,
               &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                  size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          /*
          std::cout << std::setprecision(5) << "x\ty\tyaw\ts\td\tspeed\n"
                    << car_x << "\t" << car_y << "\t" << car_yaw << "\t"
                    << car_s << "\t" << car_d << "\t" << car_speed << std::endl;
          */
          const double loop_t = 0.02;  // sec
          ego.update(car_s, car_d, car_speed * toM_S, loop_t);
          ego.lane = ego.choose_next_state(sensor_fusion);

          // check front distance of ego-vehicle
          bool brake = false;
          for (unsigned int i = 0; i < sensor_fusion.size(); ++i) {
            double sensor_d = sensor_fusion[i][6];
            // check vehicle in the same lane
            if ((int)(sensor_d / ego.lane_width) == ego.lane) {
              double sensor_vx = sensor_fusion[i][3];
              double sensor_vy = sensor_fusion[i][4];
              double sensor_s = sensor_fusion[i][5];
              // future position at next time step
              sensor_s +=
                  sqrt(sensor_vx * sensor_vx + sensor_vy * sensor_vy) * loop_t;
              // check ones that in front of ego vehicle
              if ((sensor_s > car_s) &&
                  (sensor_s - car_s < ego.preferred_buffer)) {
                brake = true;
                break;
              }
            }
          }

          unsigned int previous_path_size = previous_path_x.size();
          for (unsigned int i = 0; i < previous_path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          // find 5 points to create spline
          vector<double> Xs, Ys;
          // the heading at the end of previous path
          double yaw_rad;
          if (previous_path_size >= 2) {
            // add X[-2], Y[-2]
            // std::cout << "previous_path_size >= 2" << std::endl;
            Xs.push_back(previous_path_x[previous_path_size - 2]);
            Ys.push_back(previous_path_y[previous_path_size - 2]);
            // add X[-1], Y[-1]
            Xs.push_back(previous_path_x[previous_path_size - 1]);
            Ys.push_back(previous_path_y[previous_path_size - 1]);
            // calculate yaw
            yaw_rad = atan2(Ys[1] - Ys[0], Xs[1] - Xs[0]);
          } else {
            // create a fake previous point
            // std::cout << "previous_path_size less than 2" << std::endl;
            Xs.push_back(car_x - cos(deg2rad(car_yaw)));
            Xs.push_back(car_x);
            Ys.push_back(car_y - sin(deg2rad(car_yaw)));
            Ys.push_back(car_y);
            yaw_rad = deg2rad(car_yaw);
          }
          // rest point using map waypoints which is 30m apart in s in Frenet
          // coordinate
          // push another 3 point for spline creating
          double target_d = (double)(4 * ego.lane + 2);
          for (unsigned int i = 1; i < 4; ++i) {
            vector<double> map_waypoint_plus_lane =
                getXY(car_s + 30.0 * i, target_d, map_waypoints_s,
                      map_waypoints_x, map_waypoints_y);
            Xs.push_back(map_waypoint_plus_lane[0]);
            Ys.push_back(map_waypoint_plus_lane[1]);
          }
          // printVector("Xs= ", Xs);
          // printVector("Ys= ", Ys);
          // transfer to local coordinate
          double x_trans = Xs[0];
          double y_trans = Ys[0];
          for (unsigned int i = 0; i < Xs.size(); ++i) {
            Eigen::Vector2d local_pose = rotation2d(
                translation({-x_trans, -y_trans}, {Xs[i], Ys[i]}), -yaw_rad);
            Xs[i] = local_pose(0);
            Ys[i] = local_pose(1);
          }
          // printVector("Xs= ", Xs);
          // printVector("Ys= ", Ys);
          // create spline
          tk::spline s(Xs, Ys);
          double spline_dist, next_x, next_y;
          double next_waypoint_s = 30.0;
          spline_dist = distance(Xs[1], Ys[1], Xs[1] + next_waypoint_s,
                                 s(Xs[1] + next_waypoint_s));
          // 50 MPH for 0.02 second is 0.447 meter, use 99% and get 0.443 meter
          // x_delta will be less for busy traffic and lower lane speed
          next_x = Xs[1];
          for (unsigned int i = 0; i < 50 - previous_path_size; ++i) {
            if (brake) {
              ego.cmd_vel -= ego.max_acceleration * loop_t;
            } else {
              ego.cmd_vel += ego.max_acceleration * loop_t;
              if (ego.cmd_vel > ego.lane_speed) ego.cmd_vel = ego.lane_speed;
            }
            double x_delta =
                next_waypoint_s / (spline_dist / (ego.cmd_vel * loop_t));
            if (x_delta < 0) {
              std::cout << "x_delta < 0: " << x_delta << std::endl;
              x_delta = 0.001;
            }
            next_x += x_delta;
            // find corresponded y in spline
            next_y = s(next_x);
            // transfer from local coordinate to map coordinate
            Eigen::Vector2d next;
            next << next_x, next_y;
            Eigen::Vector2d map_pose =
                translation({x_trans, y_trans}, rotation2d(next, yaw_rad));
            next_x_vals.push_back(map_pose(0));
            next_y_vals.push_back(map_pose(1));
          }
          // printVector("x= ", next_x_vals);
          // printVector("y= ", next_y_vals);
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
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