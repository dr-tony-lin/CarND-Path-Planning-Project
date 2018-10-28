#ifndef _UTILS_CONFIG_H_
#define _UTILS_CONFIG_H_
#include <string>
#include <vector>
#include <fstream>
#include <streambuf>
#include "json.hpp"

class Config {
public:
  /**
   * Past trajectory length
   */
  static size_t pastTrajectoryLength;

  /**
   * The timesteps to predict
   */
  static size_t N;

  /**
   * The timestep in second
   */ 
  static double dt;

  /**
   * Control latency in millies
   */ 
  static long latency;

  /**
   * look ahead prediction time in seconds
   */ 
  static double lookahead;
  
  /**
   * The maximal polynomial fitting order
   */
  static int maxFitOrder;

  /**
   * The maximal polynomial fit error
   */
  static double maxFitError;

  /** 
   * Maximal steering angle
   */ 
  static double maxSteering;

  /**
   * Maximal acceleration of vehicle, m
   */ 
  static double maxAcceleration;
  static double maxJerk;
  static double laneWidth;
  static double minSafeDistance;
  static double safeDistance(const double relativeSpeed);

  static double maxDistance;
  static double minSafeGap;

  /**
   * Maximal deceleration of vehicle
   * */
  static double maxDeceleration;

  /**
   * Maximal speed of vehicle, default 80.45 (50Mph)
   */ 
  static double maxSpeed;
  
  /**
   * Minimal yaw
   */ 
  static double yawLow;
  
  /**
   * Maximal yaw
   */ 
  static double yawHigh;

  /**
   * Steering angle adjustment threshold
   */ 
  static double steerAdjustmentThresh;

  /**
   * Steering angle adjustment ratio with respect to yaw change
   */ 
  static double steerAdjustmentRatio;

  /**
   * Length from the front wheels to the center of the back wheels
   */ 
  static double Lf;

  /**
   * Cost weights: 0: cte, 1: epsi, 2: v, 3: delta, 4: delta delta, 5: not used,
   * 6: a, 7: delta a, 8: large deceleration low velocity, 9: negative speed, 10: out of range epsi
   */ 
  static std::vector<double> weights;

  /**
   * Load configuration from file
   * @param fileName name of the config file 
   */ 
  static void load(std::string fileName);
};

#endif