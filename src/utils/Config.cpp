#include <algorithm>
#include <cmath>
#include "utils.h"
#include "Config.hpp"

size_t Config::N = 50;

double Config::dt = 0.02;
size_t pastTrajectoryLength = (size_t) (1.0 / Config::dt);
int Config::maxFitOrder = 4;
double Config::maxFitError = 0.5;
long Config::latency = 100;
double Config::lookahead = 0;
double Config::maxSteering = deg2rad(33.75);
double Config::laneWidth = 4;

/**
 * Distance that we can stop the car in 3 seconds with 60% of max brake with 1 meter error
 */
double Config::minSafeDistance(const double relativeSpeed) {
  if (relativeSpeed < 0) {
    return 1.0;
  }
  return relativeSpeed * 3 - 0.5 * 0.6 * Config::maxAcceleration * 3 * 3 + 1;
}

double Config::minSafeGap = 0.3;
double Config::maxAcceleration = MpH2MpS(10);
double Config::maxJerk = MpH2MpS(10);
double Config::maxDeceleration = MpH2MpS(-20);
double Config::maxSpeed = MpH2MpS(50);
double Config::Lf = 2.67;
double Config::steerAdjustmentThresh = 0.6;
double Config::steerAdjustmentRatio = 0.025;
std::vector<double> Config::weights = {100, 100, 1, 1, 1, 5000, 1, 1000};
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
  latency = js["latency"];
  lookahead = latency * 1.0E-3;
  maxFitOrder= js["max polynomial fitting order"];
  maxFitError= js["max polynomial fitting error"];
  Lf = js["Lf"];
  steerAdjustmentThresh = js["steer adjustment threshold"];
  steerAdjustmentRatio = js["steer adjustment ratio"];
  steerAdjustmentRatio = clamp(steerAdjustmentRatio, 0.0, 0.1);
  std::vector<double> w = js["weights"];
  weights = w;
}