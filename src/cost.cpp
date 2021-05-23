#include "cost.h"

#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>

using std::string;
using std::vector;

/**
 * TODO: change weights for cost functions.
 */
const float REACH_GOAL = 0;
const float EFFICIENCY = 100;

// Here we have provided two possible suggestions for cost functions, but feel
//   free to use your own! The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be very useful in
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.

float goal_distance_cost(const Vehicle &vehicle,
                         const vector<Vehicle> &trajectory,
                         const nlohmann::json &predictions,
                         map<string, float> &data) {
  // Cost increases based on distance of intended lane (for planning a lane
  //   change) and final lane of trajectory.
  // Cost of being out of goal lane also becomes larger as vehicle approaches
  //   goal distance.
  // This function is very similar to what you have already implemented in the
  //   "Implement a Cost Function in C++" quiz.
  float cost;
  float distance = data["distance_to_goal"];
  if (distance > 0) {
    cost = 1 - 2 * exp(-(abs(2.0 * vehicle.goal_lane - data["intended_lane"] -
                             data["final_lane"]) /
                         distance));
  } else {
    cost = 1;
  }

  return cost;
}

float inefficiency_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const nlohmann::json &predictions,
                        map<string, float> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane
  //   that have traffic slower than vehicle's target speed.
  float proposed_speed_intended =
      lane_speed(vehicle, predictions, data["intended_lane"]);
  string state = trajectory.back().state;
  // when generating PLCL & PLCR trajectory, trajectory keep current lane
  // speed when no vehicle found in the intended lane. However the kinematics
  // is updated so the next speed will be different (constraint by acceleration
  // limit)
  float proposed_speed_final = lane_speed(vehicle, predictions, data["final_lane"]);

  float cost = (2.0 * vehicle.target_speed - proposed_speed_intended -
                proposed_speed_final) /
               vehicle.target_speed;

  return cost;
}

float lane_speed(const Vehicle &vehicle, const nlohmann::json &predictions,
                 int lane) {
  Vehicle temp = vehicle;
  int vehicle_ahead_id;
  if (temp.get_vehicle_ahead(predictions, lane, vehicle_ahead_id)) {
    float vx = predictions[vehicle_ahead_id][3];
    float vy = predictions[vehicle_ahead_id][4];
    return sqrt(vx * vx + vy * vy);
  }
  return vehicle.target_speed;
}

float calculate_cost(const Vehicle &vehicle, const nlohmann::json &predictions,
                     const vector<Vehicle> &trajectory) {
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, float> trajectory_data =
      get_helper_data(vehicle, trajectory, predictions);
  float cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<float(const Vehicle &, const vector<Vehicle> &,
                             const nlohmann::json &, map<string, float> &)>>
      cf_list = {inefficiency_cost};
  vector<float> weight_list = {EFFICIENCY};

  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i] * cf_list[i](vehicle, trajectory,
                                                 predictions, trajectory_data);
    cost += new_cost;
  }

  return cost;
}

map<string, float> get_helper_data(const Vehicle &vehicle,
                                   const vector<Vehicle> &trajectory,
                                   const nlohmann::json &predictions) {
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory.
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help
  //   differentiate between planning and executing a lane change in the
  //   cost functions.
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else {
    intended_lane = trajectory_last.lane;
  }

  float distance_to_goal = vehicle.goal_s - trajectory_last.s;
  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;

  return trajectory_data;
}