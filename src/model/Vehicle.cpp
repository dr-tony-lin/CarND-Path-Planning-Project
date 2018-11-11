#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>
#include <iterator>
#include "../utils/utils.h"
#include "Vehicle.hpp"
#include "Road.hpp"

using namespace std;

std::vector<double> changeTrajectoryX = {0.0, 0.05, 0.45, 0.95, 1.0};
std::vector<double> changeTrajectoryY = {0.0, 0.00, 0.50, 1.00, 1.0};
bool Vehicle::initialized = false;
tk::spline Vehicle::changeTrajectory;

void Vehicle::initialize() {
  changeTrajectory.set_points(changeTrajectoryX, changeTrajectoryY);
  initialized = true;
  for (double s = 0; s <= 1; s += 0.01) {
    cout << ",[" << s << "," << changeTrajectory(s) << "]" << endl;
  }
}

Vehicle::Vehicle(const vector<double> &data, const double _width): id(int(data[0])), x(data[1]), y(data[2]),
                    s(data[5]), d(data[6]), v(length(data[3], data[4])), width(_width) {
  if (!initialized) {
    initialize();
  }
  if (v < EPSILON) {
    v = dx = dy = 0;
  }
  else {
    dx = data[2] / v;
    dy = data[3] / v;
  }
  initFrenet();
}

Vehicle::Vehicle(const double _x, const double _y, const double _s, const double _d, const double yaw,
                 const double _v, const double _width): 
          x(_x), y(_y), s(_s), d(_d), v(min(MpH2MpS(_v), Road::current().getSpeedLimit())), width(_width) {
  if (!initialized) {
    initialize();
  }
  double rad = deg2rad(yaw);
  dx = cos(rad);
  dy = sin(rad);
  initFrenet();
  laneLeft = Road::current().dToLane(d - width * 0.5);
  laneRight = Road::current().dToLane(d + width * 0.5);
}

Vehicle::Vehicle(const Vehicle &another) {
    id = another.id;
    x = another.x;
    y = another.y;
    s = another.s;
    d = another.d;
    dx = another.dx;
    v = another.v;
    dy = another.dy;
    ax = another.ax;
    ay = another.ay;
    vs = another.vs;
    vd = another.vd;
    as = another.as;
    ad = another.ad;
    yaw = another.yaw;
    width = another.width;
    Lf = another.Lf;
    maxAcceleration = another.maxAcceleration;
    distanceToTarget = another.distanceToTarget;
    laneLeft = another.laneLeft;
    laneRight = another.laneRight;
}

Vehicle & Vehicle::operator=(const Vehicle &another) {
    id = another.id;
    x = another.x;
    y = another.y;
    s = another.s;
    d = another.d;
    v = another.v;
    dx = another.dx;
    dy = another.dy;
    ax = another.ax;
    ay = another.ay;
    vs = another.vs;
    vd = another.vd;
    as = another.as;
    ad = another.ad;
    yaw = another.yaw;
    width = another.width;
    Lf = another.Lf;
    maxAcceleration = another.maxAcceleration;
    distanceToTarget = another.distanceToTarget;
    laneLeft = another.laneLeft;
    laneRight = another.laneRight;
}

void Vehicle::initFrenet() {
    vector<double> vf = Road::current().getFrenetDirection(x, y, s, d, dx, dy);
    vs = vf[0] * v;
    vd = vf[1] * v;
    as = vf[0] * a;
    ad = vf[1] * a;
}

void Vehicle::setLf(double lf) { 
    Lf = lf;
    if (len < Lf * 1.5) {
        len = Lf * 1.5;
    }
}

vector<vector<double>> Vehicle::generatePredictions(int horizon) {
  return generatePredictions(s, d, v, [this](double t)->double {return a;}, d, s, 10, d); // 20 is not used as there is no lane change
}

vector<vector<double>> Vehicle::generatePredictions(double s, double d, double v, std::function<double (double)> acelFunc, double new_d,
                                                    double start_s, double start_d, double change_distance, int horizon) {
  vector<vector<double>> predictions;
  double lane = Road::current().dToLane(new_d);
  new_d = Road::current().laneToCenterD(lane);
  double drange = (new_d - start_d);
  bool laneChange = abs(drange) > Road::current().getLaneWidth() - EPSILON;
  double dd = (new_d - d) * 0.3 * v * Config::dt;
  double scale_d = 0;
  if (laneChange) { // lane change
    scale_d = drange;
  }
  v = min(v, Road::current().getSpeedLimit());
  double next_d = d;
  double next_v = v;
  for (int i = 1; i <= horizon; i++) {
    double t = i * Config::dt;
    double accel = acelFunc(t);
    double aa = a + accel;
    if (aa >= 0) {
      aa = min(Config::maxJerk, aa);
    }
    else {
      aa = max(-Config::maxJerk, aa);
    }
    next_v += aa * Config::dt;
    next_v = max(0.0, min(next_v, Road::current().getSpeedLimit()));
    double next_s = sAfter(s, next_v, aa, t);
    if ((drange > 0 && next_d < new_d) || (drange < 0 && next_d > new_d)) {
      if (laneChange) {
        next_d = start_d + changeTrajectory(Road::current().distanceS(next_s, start_s)/change_distance) * drange;
      }
      else {
        next_d += dd;
      }
    }
    vector<double> xy = Road::current().getXY(next_s, next_d);
    predictions.push_back({lane, xy[0], xy[1], next_s, next_d, next_v, aa});
  #ifdef DEBUG_OUT
    cout << "Trajectory t: " << t << " a: " << a << " s: " << next_s << " v: " << next_v << " d: " << next_d << " accel: " << accel << endl;
  #endif
  }
  return predictions;
}

bool Vehicle::onSameLane(const double my_d, const double another_d, const Vehicle &another) {
    double l = my_d - width * 0.5 - Config::minSafeGap;
    double r = l + width + 2 * Config::minSafeGap;
    double l2 = another_d - another.width * 0.5;
    double r2 = l2 + another.width;
    return r > l2 && l < r2;
}

bool Vehicle::hasCollision(Vehicle &another) {
    return onSameLane(d, another.d, another) && fabs(Road::current().distanceS(s, another.s)) - max(len, another.len) < 0;
}

bool Vehicle::tooClose(const double my_s, const double my_d, const double my_v, 
                        const double another_s, const double another_d, const double another_v, const Vehicle &another) {
  if (fabs(Road::current().distanceS(my_s - len, another_s)) < Config::safeDistance(another_v - my_v)) {
    return onSameLane(my_d, another_d, another);
  } 

  return false;
}

void Vehicle::globalToVehicle(vector<double> &x_trajectory, vector<double> &y_trajectory) const {
  for (size_t i = 0; i < x_trajectory.size(); i++) {
    double vx = x_trajectory[i] - this->x;
    double vy = y_trajectory[i] - this->y;
    x_trajectory[i] = vx * dx + vy * dy;
    y_trajectory[i] = vy * dx - vx * dy;
  }
}

void Vehicle::globalToVehicle(double &x, double &y) {
  double vx = x - this->x;
  double vy = y - this->y;
  x = vx * dx + vy * dy;
  y = vy * dx - vx * dy;
}

void Vehicle::vehicleToGlobal(vector<double> &x_trajectory, vector<double> &y_trajectory) const {
  for (size_t i = 0; i < x_trajectory.size(); i++) {
    double gx = this->x + x_trajectory[i] * dx - y_trajectory[i] * dy;
    double gy =this->y + x_trajectory[i] * dy + y_trajectory[i] * dx;
    x_trajectory[i] = gx;
    y_trajectory[i] = gy;
  }
}

void Vehicle::vehicleToGlobal(double &x, double &y) const {
  double gx = this->x + x * dx - y * dy;
  double gy = this->y + x * dy + y * dx;
  x = gx;
  y = gy;
}
