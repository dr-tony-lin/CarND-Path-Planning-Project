#include <algorithm>
#include <cmath>
#include "utils.h"
#include "Config.hpp"

size_t Config::N = 8;
double Config::dt = 0.02;
size_t pastTrajectoryLength = (size_t) (1.0 / Config::dt);
int Config::maxFitOrder = 4;
double Config::maxFitError = 0.5;
double Config::maxSteering = deg2rad(33.75);
double Config::laneWidth = 4;

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

long Config::minLaneChangeFreezeTime = 1;
int Config::numberOfLanes = 3;
int Config::targetLane = 1;
double Config::minSafeDistance = 15.0;
double Config::minLaneChangeSpeedGain = MpH2MpS(1.5);
double Config::minLaneChangeTime = 3.0;
double Config::minLaneChangeFrontDistance = 15.0;
double Config::minLaneChangeBackDistance = 10.0;
double Config::maxLaneChangeDistance = 60;
double Config::maxAccelerationTime = 6.0;
double Config::maxDistance = 100.0;// should be a function of current speed
double Config::minSafeGap = 0.3;
double Config::maxAcceleration = MpH2MpS(10);
double Config::maxJerk = MpH2MpS(10);
double Config::maxDeceleration = MpH2MpS(-30);
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

double Config::yawLow;
double Config::yawHigh;

void Config::load(std::string fileName) {
  std::ifstream in(fileName);

  nlohmann::json js;
  js << in;

  double speedScale;

  N = js["N"];
  dt = js["dt"];
  maxAcceleration = js["max acceleration"];
  maxAcceleration = MpH2MpS(maxAcceleration);
  maxDeceleration = js["max deceleration"];
  maxDeceleration = MpH2MpS(maxDeceleration);
  maxSteering = deg2rad(js["max steering"]);
  maxSpeed = js["max speed"];
  maxSpeed = MpH2MpS(maxSpeed);
  speedScale = maxSpeed/MpH2MpS(100.0);
  maxFitOrder= js["max polynomial fitting order"];
  maxFitError= js["max polynomial fitting error"];
  Lf = js["Lf"];
}