#ifndef _CONTROL_NAVIGATOR_HPP_
#define _CONTROL_NAVIGATOR_HPP_
#include <iostream>
#include <random>
#include <deque>
#include <vector>
#include <string>
#include "../utils/utils.h"
#include "../utils/Config.hpp"
#include "../model/Road.hpp"
#include "../model/Vehicle.hpp"
#include "Navigator.hpp"

using namespace std;

enum NavigationState {
    KL = 1,
    LCL,
    LCR,
    PLCL,
    PLCR,
    COLLIIDED
};

struct StateTransition {
    NavigationState current = NavigationState::KL;
    NavigationState target = NavigationState::KL;
    int targetLane;
    double targetS;
    double targetV;
    double minV;
    double distanceAhead;
    double distanceBehind;

    StateTransition() {};
    StateTransition(const NavigationState from, const NavigationState to, const double minSpeed, const double targetSpeed, 
                    const double dAhead, const double dBehind): current(from), target(to), minV(minSpeed), 
                    targetV(targetSpeed), distanceAhead(dAhead), distanceBehind(dBehind) {};
};

/**
 * The Navigator class control the navigation of a vehicle. It is responsible for the 
 * path planning and trajectory generation process.
 */ 
class Navigator {
protected:
  Vehicle *vehicle = NULL;
  NavigationState state = NavigationState::KL;
  StateTransition *currentTransition = NULL;

  /**
   * Navigate the vehicle and generate trajectory
   * @param fusion the sensor fusion data of other vehicle in the same side of the road
   */
  vector<vector<double>> navigate(vector<Vehicle> &fusion);

  /**
   * Find the maximum acceleration to reach the maximum safe speed in the given time interval
   * @param fusion the sensor fusion data of other vehicle in the same side of the road
   * @param lane the target lane
   * @param t the time horizon
   */
  double getMaxAcceleration(vector<Vehicle> &fusion, int lane, double t);

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
 vector<StateTransition> getLaneChangeTransitions(vector<Vehicle> &fusion, int currentLane, double t);

 void getLaneChangeTransition(vector<StateTransition> &transitions, vector<Vehicle> &fusion, bool isLeft);
  
/**
 * Sort vehicle according to their distance from the controlled vehicle
 */ 
void sortVehicle(vector<Vehicle> vehicles);

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

  bool isVehicleInitialized() { return vehicle != NULL;}

  Vehicle *getVehicle() { return vehicle;};

  void update(const std::vector<double>& previous_x, const std::vector<double>& previous_y);

  std::vector<std::vector<double>> generateTrajectory(vector<Vehicle> &fusion, int laneShift=0);

  virtual ~Navigator() {
    delete vehicle;
  }
};

#endif