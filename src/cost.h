#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(const Vehicle &vehicle, 
                     const nlohmann::json &predictions, 
                     const vector<Vehicle> &trajectory);

float goal_distance_cost(const Vehicle &vehicle,  
                         const vector<Vehicle> &trajectory,  
                         const nlohmann::json &predictions, 
                         map<string, float> &data);

float inefficiency_cost(const Vehicle &vehicle, 
                        const vector<Vehicle> &trajectory, 
                        const nlohmann::json &predictions, 
                        map<string, float> &data);

map<string, float> get_helper_data(const Vehicle &vehicle, 
                                   const vector<Vehicle> &trajectory, 
                                   const nlohmann::json &predictions);

#endif  // COST_H