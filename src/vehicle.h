#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

#include "json.hpp"

using nlohmann::json;
using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, float s, float v, float a, string state = "CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  /**
   * @brief
   *
   * @param car_x meter in map coordinate
   * @param car_y meter in map coordinate
   * @param car_s m in frenet coordinate
   * @param car_d m in frenet coordinate 0 is the center of the road
   * @param car_yaw degree
   * @param car_v m/s
   * @param loop_t delta_t in second
   */
  void update(double car_x, double car_y, double car_s, double car_d,
              double car_yaw, double car_v, double loop_t);

  int choose_next_state(nlohmann::json &predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state,
                                      nlohmann::json &predictions);

  vector<float> get_kinematics(nlohmann::json &predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(nlohmann::json &predictions);

  vector<Vehicle> lane_change_trajectory(string state,
                                         nlohmann::json &predictions);

  /**
   * @brief if we found vehicle behind current lane, keep speed. if we found the
   * speed in new land is slower, change speed, else maintain current speed.
   *
   * @param state
   * @param predictions
   * @return vector<Vehicle>
   */
  vector<Vehicle> prep_lane_change_trajectory(string state,
                                              nlohmann::json &predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(nlohmann::json &predictions, int lane, int &id);

  bool get_vehicle_ahead(nlohmann::json &predictions, int lane, int &id);

  vector<Vehicle> generate_predictions(int horizon = 2);

  void realize_next_state(vector<Vehicle> &trajectory);

  void configure(int num_lanes, int lane_width, double speed_limit,
                 double accel_limit, double max_s, double safe_dist);

  // public Vehicle variables
  struct collider {
    bool collision;  // is there a collision?
    int time;        // time collision happens
  };

  map<string, int> lane_direction = {
      {"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  int L = 1;

  float preferred_buffer;  // impacts "keep lane" behavior.

  int lane, goal_lane, lanes_available, lane_width;

  float x, y, s, d, yaw, v, a, target_speed, max_acceleration, goal_s;

  string state;

  float cmd_vel;
};

#endif  // VEHICLE_H