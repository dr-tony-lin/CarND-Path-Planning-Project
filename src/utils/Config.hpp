#ifndef _UTILS_CONFIG_H_
#define _UTILS_CONFIG_H_
#include <string>
#include <vector>
#include <fstream>
#include <streambuf>
#include "json.hpp"

class Config {
public:
  static int numberOfLanes;

  /**
   * The timesteps to predict
   */
  static size_t N;

  /**
   * The timestep in second
   */ 
  static double dt;
  
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
  static int targetLane;
  static double laneWidth;
  static long minLaneChangeFreezeTime;
  static double minSafeDistance;
  static double minLaneChangeFrontDistance;
  static double minLaneChangeBackDistance;
  static double maxLaneChangeDistance;
  static double safeDistance(const double relativeSpeed);
  static double safeLaneChangeFrontDistance(const double relativeSpeed);
  static double safeLaneChangeBackDistance(const double relativeSpeed);
  static double minLaneChangeTime;
  static double maxDistance;
  static double minSafeGap;
  static double minLaneChangeCostGain;

  static double safetyCostWeight;
  static double speedCostWeight;
  static double offTargetLaneCostWeight;
  static double changetLaneCostWeight;
  static double laneTrafficCostWeight;
  static double laneFrontTrafficCostWeight;
  static double laneRearTrafficCostWeight;
  static double laneCostWeights[];

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
   * Length from the front wheels to the center of the back wheels
   */ 
  static double Lf;

  /**
   * Load configuration from file
   * @param fileName name of the config file 
   */ 
  static void load(std::string fileName);
};

#endif