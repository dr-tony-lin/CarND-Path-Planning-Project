#include <algorithm>
#include <cmath>
#include "utils.h"
#include "Config.hpp"

/**
 * Distance that we can stop the car in 1 second with 60% of max brake with 1 meter error
 */
double Config::safeDistance(const double relativeSpeed) {
  if (relativeSpeed < 0) {
    return minSafeDistance;
  }

  return std::min(maxDistance, std::max(minSafeDistance, relativeSpeed - 0.5 * Config::maxAcceleration));
}

/**
 * Distance that we can change lane safely in 5 seconds 
 */
double Config::safeLaneChangeFrontDistance(const double relativeSpeed) {
  if (relativeSpeed < 0) {
    return minSafeDistance;
  }

  // just an estimate with considering acceleration
  return std::min(maxLaneChangeDistance, std::max(minLaneChangeFrontDistance, relativeSpeed * minLaneChangeTime));
}

/**
 * Distance that we can change lane safely in 5 seconds 
 */
double Config::safeLaneChangeBackDistance(const double relativeSpeed) {
  if (relativeSpeed < 0) {
    return minSafeDistance;
  }

  // just an estimate with considering acceleration
  return std::min(maxLaneChangeDistance, std::max(minLaneChangeBackDistance, relativeSpeed * minLaneChangeTime));
}

size_t Config::N = 4;
double Config::dt = 0.02;
size_t pastTrajectoryLength = (size_t) (1.0 / Config::dt);
double Config::laneWidth = 4;
long Config::minLaneChangeFreezeTime = 1;
int Config::numberOfLanes = 3;
int Config::targetLane = 1;
double Config::maxDistance = 80;
double Config::minSafeDistance = 15.0;
double Config::minLaneChangeTime = 3.0;
double Config::minLaneChangeFrontDistance = 15.0;
double Config::minLaneChangeBackDistance = 10.0;
double Config::maxLaneChangeDistance = 60;
double Config::minSafeGap = 0.3;
double Config::minLaneChangeCostGain = 0.1;
double Config::maxAcceleration = MpH2MpS(10);
double Config::maxJerk = MpH2MpS(10);
double Config::maxDeceleration = MpH2MpS(-40);
// The simulator use a different approach for the road trajectory than our spline approach
// They will result in slight difference in speed calculation, we need some margin to accommodate the differences
double Config::maxSpeed = MpH2MpS(75);
double Config::Lf = 2.67;

// COst function weights
double Config::safetyCostWeight = 3.0; // safety is the most important
double Config::speedCostWeight = 1.5; // faster is better
double Config::offTargetLaneCostWeight = 0.2; // prefer to go back to the target
double Config::changetLaneCostWeight = 0.2; // just for tie breaking
double Config::laneTrafficCostWeight = 1.5; // prefer no traffic
double Config::laneFrontTrafficCostWeight = 0.95; // front is important
double Config::laneRearTrafficCostWeight = 0.05; // rear is less
double Config::laneCostWeights[] = {0.5, 0.3, 0.1};

void Config::load(std::string fileName) {
  std::ifstream in(fileName);

  nlohmann::json js;
  js << in;

  double speedScale;

  N = js["N"];
  dt = js["dt"];
  Lf = 2.67;
  pastTrajectoryLength = (size_t) (1.0 / Config::dt);
  laneWidth = js["lane width"];
  numberOfLanes = js["number of lanes"];
  targetLane = js["target lane"];
  maxDistance = js["max fusion distance"];;
  maxSpeed = MpH2MpS(js["max speed"]);
  minLaneChangeFreezeTime = js["min lane change time"];
  minSafeDistance = js["mis safe distance"];
  minLaneChangeTime = js["min lane change time"];
  minLaneChangeFrontDistance = js["min lane change front distance"];
  minLaneChangeBackDistance = js["min lane change back distance"];
  maxDistance = js["max distance"];
  minSafeGap = js["min safe gap"];
  minLaneChangeCostGain = js["min lane change cost gain"];
  maxAcceleration = MpH2MpS(js["max acceleration"]);
  maxJerk = MpH2MpS(js["max jerk"]);
  maxDeceleration = MpH2MpS(js["max deceleration"]);
  safetyCostWeight = js["safety cost weight"];
  speedCostWeight = js["speed cost weight"];
  offTargetLaneCostWeight = js["off target cost weight"];
  changetLaneCostWeight = js["change lane cost weight"];
  laneTrafficCostWeight = js["lane traffic cost weight"];
  laneFrontTrafficCostWeight = js["lane front traffic cost weight"];
  laneRearTrafficCostWeight = js["lane rear traffic cost weight"];
  if (js["lane cost weight"] != NULL) {
    std::vector<double> lcw = js["lane cost weight"];
    for (size_t i = 0; i < lcw.size(); i++) {
      laneCostWeights[i] = lcw[i];
    }
  }
}