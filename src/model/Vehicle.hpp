#ifndef _MODEL_VEHICLE_HPP_
#define _MODEL_VEHICLE_HPP_
#include <iostream>
#include <random>
#include <deque>
#include <vector>
#include <map>
#include <string>
#include "../utils/utils.h"
#include "../utils/Config.hpp"
#include "Road.hpp"

using namespace std;

enum LaneBehavior {
    KL = 1,
    LCL,
    LCR,
    PLCL,
    PLCR
};

class Vehicle {
public:
  double x;
  double y;
  double dx;
  double dy;
  double v;
  double a = 0;
  double ax = 0;
  double ay = 0;
  double s;
  double d;
  double vs;
  double vd;
  double as;
  double ad;
  double yaw;
  double width = 2;
  int L = Config::Lf;
  double maxAcceleration = Config::maxAcceleration;

  Vehicle() {};

  /**
   * Constructor, takes a vector of: id, x, y, vx, vy, s, d
   */ 
  Vehicle(const std::vector<double> &data);

  /**
   * Constructor
   */ 
  Vehicle(const double x, const double y, const double s, const double d, const double yaw, const double v);

  /**
   * Assignment operator
   */ 
  Vehicle &operator=(const Vehicle &another);

  /**
   * Copy constructor
   */ 
  Vehicle(const Vehicle &another);

  void initFrenet() {
      vector<double> vf = Road::getCurrentRoad().getFrenetDirection(x, y, s, d, dx, dy);
      vs = vf[0] * v;
      vd = vf[1] * v;
      as = vf[0] * a;
      ad = vf[1] * a;
  };

  /**
   * Set length of the vehicle between the front and back wheels
   * @param length the length
   */ 
  void setLength(double length) { this->L = length;}

  /**
   * Get length of the vehicle between the front and back wheels
   */  
  double getLength() const { return L;}

  /**
   * Get the x coordinate of the vehicle
   */  
  double getX() const { return x;}

  /**
   * Get the y coordinate of the vehicle
   */  
  double getY() const { return y;}

  /**
   * Get the orientation of the vehicle
   */  
  vector<double> getOrientation() const { return {dx, dy};}

  /**
   * Get the velocity of the vehicle
   */  
  double getVelocity() const { return v;}

  /**
   * Get the steering angle of the vehicle
   */  
  double getAcceleration() const { return a;}

  /**
   * Return if another vehicle is too close this vehicle
   * @param another
   */
  bool tooClose(const Vehicle &another);

  /**
   * Get the s coordinate after t seconds
   */ 
  double dAfter(const double t) {
    return dAfter(d, vd, ad, t);
  };

  /**
   * Get the s coordinate after t seconds
   */ 
  double dAfter(const double d, const double vd, const double ad, const double t) {
    return d + vd*t + 0.5*ad*t*t;
  };

  /**
   * Get the s coordinate after t seconds
   */ 
  double sAfter(const double t) {
    return sAfter(s, vs, as, t);
  }
  
  /**
   * Get the s coordinate after t seconds
   */ 
  double sAfter(const double s, const double vs, const double as, const double t) {
    return s + vs*t + 0.5*as*t*t;
  }

  /**
   * Return the velocity after time t
   */ 
  double vAfter(const double t) {
    return vAfter(v, a, t);
  }

  /**
   * Return the velocity after time t
   */ 
  double vAfter(const double v, const double a, const double t) {
    return min(v + a * t, Road::getCurrentRoad().getSpeedLimit());
  }

  /**
   * Convert the global trajectory to the vehicle's coordinate
   * @param x_trajectory the x trajectory
   * @param y_trajectory the y trajectory
   */ 
  void globalToVehicle(std::vector<double> &x_trajectory, std::vector<double> &y_trajectory) const;

  /**
   * Convert the global coordinate to the vehicle's coordinate
   * @param x the x coordinate
   * @param y the y coordinate
   */ 
  void globalToVehicle(double &x, double &y);
                              
  /**
   * Convert the trajectory in the vehicle's coordinate to the global coordinate
   * @param x_trajectory the x trajectory
   * @param y_trajectory the y trajectory
   */ 
  void vehicleToGlobal(std::vector<double> &x_trajectory, std::vector<double> &y_trajectory) const;
  
  /**
   * Convert the point from the vehicle's coordinate to the global coordinate
   * @param x the x coordinate
   * @param y the y coordinate
   */ 
  void vehicleToGlobal(double &x, double &y) const;

  /**
   * Find the vehicle in front of the vehicle
   * @param prediction the predictions of other vehicle in the same side of the road
   * @param lane the target lane
   * @param t the time in the future to predict
   */ 
  Vehicle* getVehicleBehindAt(vector<Vehicle> &fusion, int lane, double t);
  
  /**
   * Find the vehicle in front of the vehicle
   * @param prediction the predictions of other vehicle in the same side of the road
   * @param lane the target lane
   * @param t the time in the future to predict
   */ 
  Vehicle* getVehicleAheadAt(vector<Vehicle> &fusion, int lane, double t);


  /**
   * Find the maximum acceleration to reach the maximum safe speed in the given time interval
   * @param prediction the predictions of other vehicle in the same side of the road
   * @param lane the target lane
   * @param t the time horizon
   */
  double getMaxAcceleration(vector<Vehicle> &fusion, double t) {
    return getMaxAcceleration(fusion, Road::getCurrentRoad().dToLane(d), t);
  }

  /**
   * Find the maximum acceleration to reach the maximum safe speed in the given time interval
   * @param prediction the predictions of other vehicle in the same side of the road
   * @param lane the target lane
   * @param t the time horizon
   */
  double getMaxAcceleration(vector<Vehicle> &fusion, int lane, double t);
  
  /**
   * Generate constant velocity (with respoect to s and d) trajectory
   * @param horizon the horizon step, each step is Config::dt duration
   */ 
  std::vector<std::vector<double>> constantVelocityTrajectory(double velocity, int horizon = Config::N) ;

  /**
   * Generate trajectory that will keep the current lane with the current kinematic
   * @param horizon the horizon step, each step is Config::dt duration
   */ 
  std::vector<vector<double>> generatePredictions(double s, double d, double v, double a, int horizon = Config::N);
  
  std::vector<vector<double>> generatePredictions(int horizon = Config::N);
  
  /**
  * Destructor
  */
  virtual ~Vehicle() {};
};

class Navigator {
protected:
  Vehicle *vehicle = NULL;
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