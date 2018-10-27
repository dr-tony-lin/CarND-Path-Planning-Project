#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>
#include <iterator>
#include "Vehicle.hpp"
#include "../utils/utils.h"
#include "Road.hpp"

using namespace std;

void Navigator::initializeVehicle(const double x, const double y, const double s, const double d,
                        const double yaw, const double v, const double LF, const double maxAccel) {
    vehicle = new Vehicle(x, y, s, d, yaw, v);
    vehicle->setLength(LF);
    vehicle->maxAcceleration = maxAccel;
    predictions.clear();
}

void Navigator::update(const vector<double>& previous_x, const vector<double>& previous_y) {
    if (vehicle != NULL) {
        vehicle->x = predictions.back()[1];
        vehicle->y = predictions.back()[2];
        vehicle->s = predictions.back()[3];
        vehicle->d = predictions.back()[4];
        vehicle->v = predictions.back()[5];
        vector<double> last2 = predictions[predictions.size() - 2];
        vehicle->dx = vehicle->x - last2[1];
        vehicle->dy = vehicle->y - last2[2];
        vehicle->a = predictions.back()[6];
        vehicle->initFrenet();
        for (unsigned i = 0; i < Config::N - previous_x.size() && predictions.size() > 0; i++) {
            predictions.pop_front();
        }
    }
}

vector<vector<double>> Navigator::generateTrajectory(vector<Vehicle> &fusion, int laneShift) {
  // TODO check if the lane shift is legal
  double newD = vehicle->d + laneShift * Config::laneWidth;
  int lane = Road::getCurrentRoad().dToLane(newD);
  int horizon = (Config::N - predictions.size());
  double maxAccel = vehicle->getMaxAcceleration(fusion, lane, horizon * Config::dt);
  vector<vector<double>> preds = vehicle->generatePredictions(vehicle->s, vehicle->d, vehicle->v, maxAccel, horizon);
  for (auto it: preds) {
      predictions.push_back(it);
  }

  vector<double> X;
  vector<double> Y;

  for (auto it: predictions) {
      X.push_back(it[1]);
      Y.push_back(it[2]);
  }

#ifdef VERBOSE_OUT
  cout <<  "lane: " << predictions[0][0] <<  " x: " <<  predictions[0][1] << " y: " << predictions[0][2]  << " s: " << predictions[0][3] 
       << " d: " << predictions[0][4] << " v: " << MpS2MpH(predictions[0][5]) << endl;
  cout <<  "lane: " << predictions.back()[0] << " x: " << predictions.back()[1] << " y: " << predictions.back()[2] << " s: " 
       << predictions.back()[3] << " d: " << predictions.back()[4] << " v: " << MpS2MpH(predictions.back()[5]) << endl;
#endif

  return {X, Y};
} 

Vehicle::Vehicle(const vector<double> &data): x(data[1]), y(data[2]), s(data[5]), d(data[6]), 
                    v(MpH2MpS(length(data[3], data[4]))) {
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
          x(_x), y(_y), s(_s), d(_d), v(MpH2MpS(_v)) {
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
    L = another.L;
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
    L = another.L;
    maxAcceleration = another.maxAcceleration;
}

vector<vector<double>> Vehicle::generatePredictions(int horizon) {
  return generatePredictions(s, d, v, a);
}

vector<vector<double>> Vehicle::generatePredictions(double s, double d, double v, double a, int horizon) {
  vector<vector<double>> predictions;
  double lane = Road::getCurrentRoad().dToLane(d);
  double new_d = Road::getCurrentRoad().laneToCenterD(lane);
  double dd = (new_d - d) / horizon;
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

double Vehicle::getMaxAcceleration(vector<Vehicle> &fusion, int lane, double t) {
    Vehicle *vehicle = getVehicleAheadAt(fusion, lane, t);
    if (vehicle == NULL) {
        double accel = min(min(Road::getCurrentRoad().getSpeedLimit() - v, Config::maxJerk), maxAcceleration);
        cout << "No vehicle in front, a: " << MpS2MpH(accel) << endl;
        return accel;
    }
    else {
        double distance = vehicle->s - s - Config::minSafeDistance(v - vehicle->v);
        double newA = 2 * (distance + (vehicle->v - v) * t) / (t*t) + vehicle->a;
        if (newA - a > Config::maxJerk) {
            newA = Config::maxJerk;
        }
        else if (newA - a < -Config::maxJerk) {
            newA = -Config::maxJerk;
        }
        cout << "Vehicle ahead: " << vehicle->s << " " << vehicle->v << " ahead " << s << " " << v << " a: " << MpS2MpH(newA) << endl;
        return newA;
    }
}

Vehicle* Vehicle::getVehicleBehindAt(vector<Vehicle> &fusion, int lane, double t) {
    int max_s = -1;
    double nextS = sAfter(t);
    Vehicle *vehicle = NULL;
    double otherVehicleTime = Config::N * Config::dt;
    for (Vehicle &temp_vehicle: fusion) {
        int temp_lane = Road::getCurrentRoad().dToLane(temp_vehicle.dAfter(otherVehicleTime));
        if (lane == temp_lane) {
            double tempS = temp_vehicle.sAfter(otherVehicleTime);
            if (tempS < s && tempS > max_s) {
                max_s = tempS;
                vehicle = &temp_vehicle;
            }
        }
    }
    return vehicle;
}

Vehicle* Vehicle::getVehicleAheadAt(vector<Vehicle> &fusion, int lane, double t) {
    double min_s = 10000000000;
    double nextS = sAfter(t);
    cout << "Check vehicle on " << lane << " after " << t << " with s: " << nextS << endl;
    Vehicle *vehicle = NULL;
    double otherVehicleTime = t;
    for (Vehicle &temp_vehicle: fusion) {
        int temp_lane = Road::getCurrentRoad().dToLane(temp_vehicle.dAfter(otherVehicleTime));
        if (lane == temp_lane) {
            double tempS = temp_vehicle.sAfter(otherVehicleTime);
            cout << "Check vehicle on lane " << temp_lane << " with s: " << tempS << endl;
            if (tempS > nextS && tempS < min_s) {
                min_s = tempS;
                vehicle = &temp_vehicle;
            }
        }
    }
    if (vehicle != NULL) {
        cout << "Fusion x " << vehicle->x << " y " << vehicle->y << " s " << vehicle->s << " v " << vehicle->v << " Future " << min_s << endl;
    }
    return vehicle;
}

/**
 * Too close
 * @param car2
 */
bool Vehicle::tooClose(const Vehicle &another) {
  if (fabs(s - another.s) < Config::minSafeDistance(another.v - v)) {
    double l = d - width * 0.5 - Config::minSafeGap;
    double r = l + width + Config::minSafeGap;
    double l2 = another.d - another.width * 0.5;
    double r2 = l2 + another.width;
    return r > l2 && l < r2;
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
