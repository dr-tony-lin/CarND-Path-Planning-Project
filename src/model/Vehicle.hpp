#ifndef _MODEL_VEHICLE_HPP_
#define _MODEL_VEHICLE_HPP_
#include <vector>
#include <functional>
#include "../utils/Config.hpp"
#include "../utils/spline.h"
#include "Road.hpp"

using namespace std;

class Navigator;

/**
 * The Vehicle class encapsulate a vehicle
 */ 
class Vehicle {
  double distanceToTarget;
  static bool initialized;
  static tk::spline changeTrajectory;

  friend class Navigator;
public:
  int id = -1;
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
  int Lf = Config::Lf;
  int len = Config::Lf * 1.5;
  double maxAcceleration = Config::maxAcceleration;
  int laneLeft;
  int laneRight;

  Vehicle() {
    if (!initialized) {
      initialize();
    }
  };

  /**
   * Constructor, takes a vector of: id, x, y, vx, vy, s, d
   */ 
  Vehicle(const std::vector<double> &data, const double width = 2);

  /**
   * Constructor
   */ 
  Vehicle(const double x, const double y, const double s, const double d, const double yaw = 0, const double v = 0,
          const double width = 2);

  /**
   * Assignment operator
   */ 
  Vehicle &operator=(const Vehicle &another);

  /**
   * Copy constructor
   */ 
  Vehicle(const Vehicle &another);

  static void initialize();

  void initFrenet();

  /**
   * Set length of the vehicle between the front and back wheels
   * @param lf the length
   */ 
  void setLf(double lf) ;

  /**
   * Get length of the vehicle between the front and back wheels
   */  
  double getLf() const { return Lf;}

  /**
   * Set length of the vehicle
   * @param length the length
   */ 
  void setLength(double length) { this->len = length;}

  /**
   * Get length of the vehicle
   */  
  double getLength() const { return len;}

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
   * @param my_s my future s coordinate
   * @param my_d my future d
   * @param my_v my future v
   * @param another_s future s of another car
   * @param another_d future d of another car
   * @param another_v future v of another car
   * @param another another car
   */
  bool tooClose(const double my_s, const double my_d, const double my_v, 
                        const double another_s, const double another_d, const double another_v, const Vehicle &another);
  /**
   * Check if the given vehicle has collision with the vehicle under control
   */
  bool hasCollision(Vehicle &another);

  /**
   * Return if another vehicle is on the same lane as this vehicle
   * @param my_d my future d
   * @param another_d future d of another car
   * @param another another car
   */
bool onSameLane(const double my_d, const double another_d, const Vehicle &another);

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
    return Road::current().normalizeS(s + vs*t + 0.5*as*t*t);
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
    return min(v + a * t, Road::current().getSpeedLimit());
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
   * Generate constant velocity (with respoect to s and d) trajectory
   * @param horizon the horizon step, each step is Config::dt duration
   */ 
  std::vector<std::vector<double>> constantVelocityTrajectory(double velocity, int horizon = Config::N) ;

  /**
   * Generate trajectory that will keep the current lane with the current kinematic
   * @param horizon the horizon step, each step is Config::dt duration
   */ 
  std::vector<vector<double>> generatePredictions(double s, double d, double v, std::function<double (double)> a, 
                    double newD, double start_s, double start_d, double change_distance, int horizon = Config::N);
  
  std::vector<vector<double>> generatePredictions(int horizon = Config::N);
  
  /**
  * Destructor
  */
  virtual ~Vehicle() {};
};

#endif