#ifndef VEHICLE_H
#define VEHICLE_H
//#define SIM
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
   * @brief update state, calculate lane, accelerate and safe distance
   *
   * @param car_s m in frenet coordinate
   * @param car_d m in frenet coordinate 0 is the center of the road
   * @param car_v m/s
   * @param loop_t delta_t in second
   */
  void update(double car_s, double car_d, double car_v, double loop_t);

  /**
   * @brief rough position after 1 sec and calculate cost function. Update
   * traffic speed for that lane
   *
   * @param predictions sensor fusion data
   * @return int lane index of minimum cost, for trajectory generation
   */
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

  bool get_vehicle_behind(const nlohmann::json &predictions, int lane, int &id);

  bool get_vehicle_ahead(const nlohmann::json &predictions, int lane, int &id);
#ifdef SIM
  vector<Vehicle> generate_predictions(int horizon = 2);

  void realize_next_state(vector<Vehicle> &trajectory);

  // public Vehicle variables
  struct collider {
    bool collision;  // is there a collision?
    int time;        // time collision happens
  };
#endif
  void configure(int num_lanes, int lane_width, float speed_limit,
                 float accel_limit, double max_s);

  map<string, int> lane_direction = {
      {"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  float preferred_buffer;  // impacts "keep lane" behavior.

  int lane, goal_lane, lanes_available, lane_width;

  float s, v, a, target_speed, max_acceleration, goal_s;

  string state;

  float cmd_vel, lane_speed;
  float last_update_time;
};

void printVector(const string msg, const vector<double> &v);
void printVector(const string msg, const vector<float> &v);
void printVector(const string msg, const vector<string> &v);

#endif  // VEHICLE_H