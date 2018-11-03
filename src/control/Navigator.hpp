#ifndef _CONTROL_NAVIGATOR_HPP_
#define _CONTROL_NAVIGATOR_HPP_
#include <iostream>
#include <random>
#include <functional>
#include <deque>
#include <vector>
#include <string>
#include "../utils/utils.h"
#include "../utils/Config.hpp"
#include "../model/Road.hpp"
#include "../model/Vehicle.hpp"
#include "../model/State.hpp"
#include "Cost.hpp"
#include "Navigator.hpp"

using namespace std;

/**
 * The Navigator class control the navigation of a vehicle. It is responsible for the 
 * path planning and trajectory generation process.
 */ 
class Navigator {
protected:
  Vehicle *vehicle = NULL;
  NavigationState state = NavigationState::KL;
  StateTransition *currentTransition = NULL;
  CostEvaluator cost;
  long long lastLaneChangeTime;

  /**
   * Generate trajectory to the target lane
   */ 
  std::vector<std::vector<double>> generateTrajectory(const vector<Vehicle> &fusion, const double laneChangeDistance);

  /**
   * Find the acceleration for the transition
   */
  std::function<double (double)> getAcceleration(StateTransition &transition);

  /**
   * Find vehicles on collision course
   * @param fusion the sensor fusion data of other vehicle in the same side of the road
   * @param lane the target lane
   * @param t the time in the future to predict
   */ 
  std::vector<Vehicle*> findVehicleOnCollisionCourse(vector<Vehicle> &fusion, int lane, double t, double safeRange=5);

  /**
   * Find the vehicle in front of the vehicle
   * @param fusion the sensor fusion data of other vehicle in the same side of the road
   * @param lane the target lane
   * @param t the time in the future to predict
   */ 
  Vehicle* getVehicleBehindAt(vector<Vehicle> &fusion, int lane, double t);

  /**
   * Find the vehicle in front of the vehicle
   * @param fusion the sensor fusion data of other vehicle in the same side of the road
   * @param lane the target lane
   * @param t the time in the future to predict
   */ 
  Vehicle* getVehicleAheadAt(vector<Vehicle> &fusion, int lane, double t);

  /**
   * Find the vehicle in front of the vehicle
   * @param fusion the sensor fusion data of other vehicle in the same side of the road
   * @param currentLane the current lane
   */ 
 vector<StateTransition> getLaneStateTransitions(vector<Vehicle> &fusion, int currentLane, double t);

 void getLaneStateTransition(vector<StateTransition> &transitions, vector<Vehicle> &fusion, const int currentLane, const int laneChange);
  
/**
 * Order insert vehicle according to their distance from the controlled vehicle
 */ 
void orderedInsert(vector<Vehicle> &vehicles, Vehicle &v);

public:
  std::deque<std::vector<double>> predictions;

  Navigator() {};

  /**
   * Initialize the vehicle
   * @param x x coordinate
   * @param y y coordinate
   * @param s the s coordinate in the map
   * @param d the d coordinate in the map
   * @param v the velocity of the vehicle
   * @param yaw the yaw angle
   * @param Lf length of the vehicle between the front and back wheels
   * @param maxAccel max acceleration of the vehicle
   */
  void initializeVehicle(const double x, const double y, const double s, const double d, const double yaw, const double v = 0,
                        const double LF=Config::Lf, const double maxAccel=Config::maxAcceleration);

  /**
   * Return true if the vehicle has been initialized
   */ 
  bool isVehicleInitialized() { return vehicle != NULL;}

  /**
   * Return the controlled vehicle
   */ 
  Vehicle *getVehicle() { return vehicle;};

  /**
   * Update the left over trajectory
   */ 
  void update(const std::vector<double>& previous_x, const std::vector<double>& previous_y);

  /**
   * Navigate the vehicle and generate trajectory
   * @param fusion the sensor fusion data of other vehicle in the same side of the road
   */
  vector<vector<double>> navigate(vector<Vehicle> &fusion);

  virtual ~Navigator() {
    delete vehicle;
  }
};

#endif