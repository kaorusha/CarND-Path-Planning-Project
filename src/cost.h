#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(const Vehicle &vehicle, const nlohmann::json &predictions,
                     const vector<Vehicle> &trajectory);

float goal_distance_cost(const Vehicle &vehicle,
                         const vector<Vehicle> &trajectory,
                         const nlohmann::json &predictions,
                         map<string, float> &data);

float inefficiency_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const nlohmann::json &predictions,
                        map<string, float> &data);

/**
 * @brief Find the vehicle ahead in that lane. If no vehicle ahead than use
 * target speed;
 *
 * @param vehicle object to call vehicle class method
 * @param predictions sensor fusion data
 * @param lane indended lane
 * @return float speed in m/s
 */
float lane_speed(const Vehicle &vehicle, const nlohmann::json &predictions,
                 int lane);

map<string, float> get_helper_data(const Vehicle &vehicle,
                                   const vector<Vehicle> &trajectory,
                                   const nlohmann::json &predictions);

#endif  // COST_H