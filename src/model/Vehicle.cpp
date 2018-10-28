#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>
#include <iterator>
#include "../utils/utils.h"
#include "Vehicle.hpp"
#include "Road.hpp"

using namespace std;

Vehicle::Vehicle(const vector<double> &data): x(data[1]), y(data[2]), s(data[5]), d(data[6]), 
                    v(min(MpH2MpS(length(data[3], data[4])), Road::getCurrentRoad().getSpeedLimit())) {
  if (v < EPSILON) {
    v = dx = dy = 0;
  }
  else {
    dx = data[2] / v;
    dy = data[3] / v;
  }
  initFrenet();
}

Vehicle::Vehicle(const double _x, const double _y, const double _s, const double _d, const double yaw, const double _v): 
          x(_x), y(_y), s(_s), d(_d), v(min(MpH2MpS(_v), Road::getCurrentRoad().getSpeedLimit())) {
  double rad = deg2rad(yaw);
  dx = cos(rad);
  dy = sin(rad);
  initFrenet();
}

Vehicle::Vehicle(const Vehicle &another) {
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
}

Vehicle & Vehicle::operator=(const Vehicle &another) {
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
}

void Vehicle::initFrenet() {
    vector<double> vf = Road::getCurrentRoad().getFrenetDirection(x, y, s, d, dx, dy);
    vs = vf[0] * v;
    vd = vf[1] * v;
    as = vf[0] * a;
    ad = vf[1] * a;
}

/**
 * Set length of the vehicle between the front and back wheels
 * @param lf the length
 */ 
void Vehicle::setLf(double lf) { 
    Lf = lf;
    if (len < Lf * 1.5) {
        len = Lf * 1.5;
    }
}

vector<vector<double>> Vehicle::generatePredictions(int horizon) {
  return generatePredictions(s, d, v, a);
}

vector<vector<double>> Vehicle::generatePredictions(double s, double d, double v, double a, int horizon) {
  vector<vector<double>> predictions;
  double lane = Road::getCurrentRoad().dToLane(d);
  double new_d = Road::getCurrentRoad().laneToCenterD(lane);
  double dd = (new_d - d) / horizon;
  v = min(v, Road::getCurrentRoad().getSpeedLimit());
  for (int i = 1; i <= horizon; i++) {
    double t = i * Config::dt;
    double next_s = sAfter(s, v, a, t);
    d += dd;
    double next_v = max(0.0, min(v + a * t, Road::getCurrentRoad().getSpeedLimit()));
    vector<double> xy = Road::getCurrentRoad().getXY(next_s, d);
    predictions.push_back({lane, xy[0], xy[1], next_s, d, next_v, a});
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
    return onSameLane(d, another.d, another) && fabs(s - another.s) - max(len, another.len) < 0;
}

bool Vehicle::tooClose(const double my_s, const double my_d, const double my_v, 
                        const double another_s, const double another_d, const double another_v, const Vehicle &another) {
  if (fabs(my_s - len - another_s) < Config::safeDistance(another_v - my_v)) {
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
