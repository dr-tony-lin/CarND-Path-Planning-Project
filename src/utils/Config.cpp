#include <algorithm>
#include <cmath>
#include "utils.h"
#include "Config.hpp"

size_t Config::N = 50;
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
    return 1.0;
  }

  return std::min(maxDistance, std::max(minSafeDistance, relativeSpeed - 0.5 * 0.8 * Config::maxAcceleration));
}

/**
 * Distance that we can change lane safely in 5 seconds 
 */
double Config::safeLaneChangeDistance(const double relativeSpeed) {
  if (relativeSpeed < 0) {
    return 1.0;
  }

  // just an estimate with considering acceleration
  return std::min(maxDistance, std::max(minLaneChangeDistance, relativeSpeed * minLaneChangeTime));
}

int Config::numberOfLanes = 3;
double Config::minSafeDistance = 1.0;
double Config::minLaneChangeTime = 5.0;
double Config::minLaneChangeDistance = 5.0;
double Config::maxDistance = 150.0;
double Config::minSafeGap = 0.3;
double Config::maxAcceleration = MpH2MpS(10);
double Config::maxJerk = MpH2MpS(8);
double Config::maxDeceleration = MpH2MpS(-20);
// The simulator use a different approach for the road trajectory than our spline approach
// They will result in slight difference in speed calculation, we need some margin to accommodate the differences
double Config::maxSpeed = MpH2MpS(47.5); 
double Config::Lf = 2.67;
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