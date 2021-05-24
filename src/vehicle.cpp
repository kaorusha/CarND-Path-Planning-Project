#include "vehicle.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include "cost.h"

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle() {
  // initial command velocity is 0.0 m/s
  this->v = 0.0;
  this->state = "CS";
  this->cmd_vel = 0.0;
  this->last_update_time = 0.2;
  this->goal_lane = -1;
}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
}

Vehicle::~Vehicle() {}

void Vehicle::update(double car_s, double car_d, double car_v, double loop_t) {
  // define lane = 0, 1, 2
  this->s = car_s;
  // calculate acceleration before update speed
  this->a = (car_v - this->v) / loop_t;
  this->v = car_v;
  this->lane = car_d / lane_width;
  // minimum decelerate distance plus a buffer in meter
  this->preferred_buffer = 0.5 * car_v * 3600/1000;
}

void printVector(const string msg, const vector<float> &v) {
  std::cout << msg;
  for (unsigned int i = 0; i < v.size(); ++i) {
    std::cout << std::setprecision(10) << v[i] << "\t";
  }
  std::cout << std::endl;
}

void printVector(const string msg, const vector<string> &v) {
  std::cout << msg;
  for (unsigned int i = 0; i < v.size(); ++i) {
    std::cout << std::setprecision(10) << v[i] << "\t";
  }
  std::cout << std::endl;
}

int Vehicle::choose_next_state(nlohmann::json &predictions) {
  /**
   * Here you can implement the transition_function code from the Behavior
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions vector. A 2d vector of cars and then that car's [car's
   * unique ID, car's x position in map coordinates, car's y position in map
   * coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s
   * position in frenet coordinates, car's d position in frenet coordinates.]
   * @output The best (lowest cost) of next ego vehicle lane.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing
   *    a vehicle trajectory, given a state and predictions. Note that
   *    trajectory vectors might have size 0 if no possible trajectory exists
   *    for the state.
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a
   * trajectory.
   *
   * TODO: Your solution here.
   */
  vector<string> possible_successor_state = successor_states();
  vector<string>::iterator state = possible_successor_state.begin();
  float min_cost = std::numeric_limits<float>::max();
  float cost_for_state;
  string next_state;
  int next_lane;
  while (state != possible_successor_state.end()) {
    // generate a rough idea of what trajectory we would
    // follow IF we chose this state.
    vector<Vehicle> trajectory_for_state =
        generate_trajectory(*state, predictions);

    // calculate the "cost" associated with that trajectory.
    if (trajectory_for_state.size() != 0) {
      cost_for_state = calculate_cost(*this, predictions, trajectory_for_state);
      if (cost_for_state < min_cost) {
        min_cost = cost_for_state;
        next_state = *state;
        next_lane = trajectory_for_state.back().lane;
        this->lane_speed = trajectory_for_state.back().v;
      }
    }
    ++state;
  }
  /**
   * TODO: Change return value here:
   */
  if (this->state != next_state) {
    std::cout << "next state= " << next_state << "\tnext lane= " << next_lane
              << std::endl;
  }
  this->state = next_state;
  return next_lane;
}

vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM
  //   discussed in the course, with the exception that lane changes happen
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if (state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCL") == 0) {
    if (lane != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (lane != lanes_available - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }

  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
                                             nlohmann::json &predictions) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }

  return trajectory;
}

vector<float> Vehicle::get_kinematics(nlohmann::json &predictions, int lane) {
  // Gets next timestep kinematics (position, velocity, acceleration)
  //   for a given lane. Tries to choose the maximum velocity and acceleration,
  //   given other vehicle positions and accel/velocity constraints.
  float max_velocity_accel_limit = this->max_acceleration + this->v;
  float new_position;
  float new_velocity;
  float new_accel;
  int vehicle_ahead_id;
  int vehicle_behind_id;

  if (get_vehicle_ahead(predictions, lane, vehicle_ahead_id)) {
    float vx = predictions[vehicle_ahead_id][3];
    float vy = predictions[vehicle_ahead_id][4];
    float vehicle_ahead_v = sqrt(vx * vx + vy * vy);
    if (get_vehicle_behind(predictions, lane, vehicle_behind_id)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      new_velocity = vehicle_ahead_v;
    } else {
      float s = predictions[vehicle_ahead_id][5];
      float max_velocity_in_front = (s - this->s - this->preferred_buffer) +
                                    vehicle_ahead_v - 0.5 * (this->a);
      new_velocity =
          std::min(std::min(max_velocity_in_front, max_velocity_accel_limit),
                   this->target_speed);
    }
  } else {
    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
  }

  new_accel = new_velocity - this->v;  // Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity + new_accel / 2.0;

  return {new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  float next_pos = position_at(1);
  vector<Vehicle> trajectory = {
      Vehicle(this->lane, this->s, this->v, this->a, this->state),
      Vehicle(this->lane, next_pos, this->v, 0, this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(nlohmann::json &predictions) {
  // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {
      Vehicle(this->lane, this->s, this->v, this->a, this->state)};
  vector<float> kinematics = get_kinematics(predictions, this->lane);
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];
  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));

  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(
    string state, nlohmann::json &predictions) {
  // Generate a trajectory preparing for a lane change.
  float new_s;
  float new_v;
  float new_a;
  int vehicle_behind_id;
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory = {
      Vehicle(this->lane, this->s, this->v, this->a, this->state)};
  vector<float> curr_lane_new_kinematics =
      get_kinematics(predictions, this->lane);

  if (get_vehicle_behind(predictions, this->lane, vehicle_behind_id)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];
  } else {
    vector<float> best_kinematics;
    vector<float> next_lane_new_kinematics =
        get_kinematics(predictions, new_lane);
    // Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));

  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
                                                nlohmann::json &predictions) {
  // Generate a lane change trajectory.
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  for (unsigned int i = 0; i < predictions.size(); ++i) {
    float sensor_d = predictions[i][6];
    if ((int)(sensor_d / this->lane_width) == new_lane) {
      float sensor_s = predictions[i][5];
      // permit lane changing if maintaining safe distance
      if (fabs(sensor_s - this->s) < this->preferred_buffer) {
        // If lane change is not possible, return empty trajectory.
        return trajectory;
      }
    }
  }
  trajectory.push_back(
      Vehicle(this->lane, this->s, this->v, this->a, this->state));
  vector<float> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(
      Vehicle(new_lane, kinematics[0], kinematics[1], kinematics[2], state));
  return trajectory;
}

void Vehicle::increment(int dt = 1) { this->s = position_at(dt); }

float Vehicle::position_at(int t) {
  return this->s + this->v * t + this->a * t * t / 2.0;
}

bool Vehicle::get_vehicle_behind(const nlohmann::json &predictions, int lane,
                                 int &id) {
  // Returns a true if a vehicle is found behind the current vehicle, false
  //   otherwise. The passed reference id is updated if a vehicle is found.
  float max_s = -1;
  bool found_vehicle = false;

  for (unsigned int i = 0; i < predictions.size(); ++i) {
    float sensor_d = predictions[i][6];
    if ((int)(sensor_d / this->lane_width) == lane) {
      int vehicle_id = predictions[i][0];
      float sensor_s = predictions[i][5];
      if (sensor_s < this->s && sensor_s > max_s) {
        max_s = sensor_s;
        id = vehicle_id;
        found_vehicle = true;
      }
    }
  }

  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(const nlohmann::json &predictions, int lane,
                                int &id) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference id is updated if a vehicle is found.
  float min_s = this->goal_s;
  bool found_vehicle = false;
  for (unsigned int i = 0; i < predictions.size(); ++i) {
    float sensor_d = predictions[i][6];
    if ((int)(sensor_d / this->lane_width) == lane) {
      int vehicle_id = predictions[i][0];
      float sensor_s = predictions[i][5];
      if (sensor_s > this->s && sensor_s < min_s) {
        min_s = sensor_s;
        id = vehicle_id;
        found_vehicle = true;
      }
    }
  }
  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
  // Generates predictions for non-ego vehicles to be used in trajectory
  //   generation for the ego vehicle.
  vector<Vehicle> predictions;
  for (int i = 0; i < horizon; ++i) {
    float next_s = position_at(i);
    float next_v = 0;
    if (i < horizon - 1) {
      next_v = position_at(i + 1) - s;
    }
    predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  }

  return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
  // Sets state and kinematics for ego vehicle using the last state of the
  // trajectory.
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
}

void Vehicle::configure(int num_lanes, int width, float speed_limit,
                        float accel_limit, double max_s) {
  // Called by simulator before simulation begins. Sets various parameters which
  //   will impact the ego vehicle.
  target_speed = speed_limit;
  lanes_available = num_lanes;
  max_acceleration = accel_limit;
  lane_width = width;
  goal_s = max_s;
}