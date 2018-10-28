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
        vehicle->a = predictions.back()[6];
        vector<double> last2 = predictions[predictions.size() - 2];
        vehicle->dx = vehicle->x - last2[1];
        vehicle->dy = vehicle->y - last2[2];
        vector<double> fNormal = unitv(vehicle->s - last2[3], vehicle->d - last2[4]);
        vehicle->vs = vehicle->v * fNormal[0];
        vehicle->vd = vehicle->v * fNormal[1];
        vehicle->as = vehicle->a * fNormal[0];
        vehicle->ad = vehicle->a * fNormal[1];
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
#ifdef DEBUG_OUT
    cout <<  "Generate prediction from  s: " << vehicle->s << " d: " << vehicle->d << " v: " << vehicle->v << " a: " << MpS2MpH(maxAccel) << " horizon: " << horizon << endl;
#endif
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
    int inc = 5;
    for (int i = 0; i < (int) predictions.size(); i += inc) {
        if (i >= (int) predictions.size() - inc && inc > 1) inc >>= 1;
        cout << i << " lane: " << predictions[i][0] <<  " x: " <<  predictions[i][1] << " y: " << predictions[i][2]  << " s: " << predictions[i][3] 
            << " d: " << predictions[i][4] << " v: " << MpS2MpH(predictions[i][5]) << endl;
    }
#endif

  return {X, Y};
} 

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

double Vehicle::getMaxAcceleration(vector<Vehicle> &fusion, int lane, double t) {
    vector<Vehicle *> onCollisionCourse = detectCollision(fusion, lane, Config::N * Config::dt, 10);
    double newV = vAfter(t); // the speed after t if we keep current accel
    double newS = sAfter(t); // the position after t if we keep current speed
    if (onCollisionCourse.size() == 0) {
        double accel = min(min((Road::getCurrentRoad().getSpeedLimit() - newV) / t, Config::maxJerk), maxAcceleration);
#ifdef DEBUG_OUT
        cout << "No vehicle, v: " << newV << " s: " << newS << " a: " << MpS2MpH(accel) << endl;
#endif
        return accel;
    }
    else {
        double result = 0;
        double frontAccel = 1000000;
        double backAccel = -1000000;
        double bestFrontV = 0;
        double bestBackV = 0;
        double bestSafeDistance = 0;
        double bestDistance = 0;
        double vehicleV = 0;
        for (auto vehicle: onCollisionCourse) {
            double collisionS = vehicle->sAfter(t);
            double safeDistance = Config::safeDistance(newV - vehicle->v);
            double distanceFront = Road::getCurrentRoad().distanceS(Road::getCurrentRoad().normalizeS(collisionS - Config::Lf * 1.5), newS);
            double distanceBack = Road::getCurrentRoad().distanceS(collisionS, Road::getCurrentRoad().normalizeS(newS - len));
#ifdef DEBUG_OUT
            cout << "Check on collision course: s: " << vehicle->s << " d: " << vehicle->d << " v: " << MpS2MpH(vehicle->v) << " vs: " << MpS2MpH(vehicle->vs)
                    << " vd: " << MpS2MpH(vehicle->vd) << " safe dist: " << safeDistance << " front dist: " << distanceFront << " back dist: " << distanceBack << endl;
#endif
            if (distanceFront > 0) {
                // use current s and v for distance
                double distance = Road::getCurrentRoad().distanceS(Road::getCurrentRoad().normalizeS(vehicle->s - Config::Lf * 1.5 - safeDistance), s); 
                double targetV = min(vehicle->v + distance / t + 0.5 * (vehicle->a - a) * t, Road::getCurrentRoad().getSpeedLimit());
                double newA = (targetV - v) / t;
                if (newA - a > Config::maxJerk) {
                    newA = Config::maxJerk;
                }
                else if (newA - a < -Config::maxJerk) {
                    newA = -Config::maxJerk;
                }
                if (targetV > Road::getCurrentRoad().getSpeedLimit() * 0.8) {
                    newA = newA * (Road::getCurrentRoad().getSpeedLimit() - targetV) / Road::getCurrentRoad().getSpeedLimit();
                }
                if (frontAccel > newA) {
                    frontAccel = newA;
                    bestFrontV = targetV;
                    bestSafeDistance = safeDistance;
                    vehicleV = vehicle->v;
                    bestDistance = distance;
#ifdef DEBUG_OUT
                    cout << "New front, v: " << newV << " safe dist to keep: " << bestSafeDistance << " distance to travel: " << bestDistance
                            << " target v: " << bestFrontV << " accel: " << MpS2MpH(newA) << endl;
#endif
                }
            }
            else if (distanceBack < 0) { // Rear collission course
                // use current s and v for distance
                double distance = -Road::getCurrentRoad().distanceS(vehicle->s, Road::getCurrentRoad().normalizeS(s - len - safeDistance)); 
                double minV = min(vehicle->v + distance / t + 0.5 * (vehicle->a - a) * t, Road::getCurrentRoad().getSpeedLimit());
                double minA = (minV - v) / t * 1.3; // 30% over what we need to avoid back collision
                if (minA - a > Config::maxJerk) {
                    minA = Config::maxJerk;
                }
                else if (minA - a < -Config::maxJerk) {
                    minA = -Config::maxJerk;
                }
                if (minV > Road::getCurrentRoad().getSpeedLimit() * 0.8) {
                    minA = minA * (Road::getCurrentRoad().getSpeedLimit() * 0.98 - minV) / Road::getCurrentRoad().getSpeedLimit();
                }
                if (backAccel < minA) {
                    backAccel = minA;
                    bestBackV = minV;
                    bestSafeDistance = safeDistance;
                    vehicleV = vehicle->v;
                    bestDistance = distance;
#ifdef DEBUG_OUT
                    cout << "New back, v: " << newV << " safe dist to keep: " << bestSafeDistance << " distance to travel: " << bestDistance
                            << " target v: " << bestBackV << " accel: " << MpS2MpH(backAccel) << endl;
#endif
                }
            }
            else {
                cout << "Error in collision detection! front: " << distanceFront << " back: " << distanceBack << endl;
            }
        }

        if (frontAccel < backAccel) { // not good
            // TODO change lane
            cout << " Sandwitched! " << endl;
            return 0.0;
        }
        else {
            if (frontAccel < 1000000 - EPSILON) { // for rounding error
                result = frontAccel;
#ifdef DEBUG_OUT
                cout << "Present front, v: " << newV << " safe dist to keep: " << bestSafeDistance << " distance to travel: " << bestDistance
                        << " target v: " << bestFrontV << " accel: " << MpS2MpH(frontAccel) << endl;
#endif
            }
            else if (backAccel > -1000000 + EPSILON) {
                result = backAccel;
#ifdef DEBUG_OUT
                cout << "Prevent back, v: " << newV << " safe dist to keep: " << bestSafeDistance << " distance to travel: " << bestDistance
                        << " target v: " << bestBackV << " accel: " << MpS2MpH(backAccel) << endl;
#endif
            }
        }
        return result;
    }
}

vector<Vehicle*> Vehicle::detectCollision(vector<Vehicle> &fusion, int lane, double t, double safeRange) {
    double nextS = sAfter(t);
    vector<Vehicle*> collisions;
#ifdef DEBUG_OUT
    cout << "Check vehicle on " << lane << " after " << t << " with s: " << nextS << endl;
#endif
    double otherVehicleTime = Config::N * Config::dt;
    for (Vehicle &temp_vehicle: fusion) {
// #ifdef VERBOSE_OUT
//         cout << "Check vehicle - s " << temp_vehicle.s << " v " << MpS2MpH(temp_vehicle.v) << " d: " << temp_vehicle.d << " vs: " << temp_vehicle.vs << " vd: " << temp_vehicle.vd
//                 << " furute: s: " << temp_vehicle.sAfter(otherVehicleTime) << " furute: d: " << temp_vehicle.dAfter(otherVehicleTime) << endl;
// #endif
        // Assume the vehicle can not cross multiple lanes in the horizon, should be fine unless the horizon is too big
        if (onSameLane(Road::getCurrentRoad().laneToCenterD(lane), temp_vehicle.dAfter(otherVehicleTime), temp_vehicle) ||
            onSameLane(d, temp_vehicle.d, temp_vehicle)) { // accept the situations that other vehicle may or may not change lane
            double tempS = temp_vehicle.sAfter(otherVehicleTime);
            double dist = Road::getCurrentRoad().distanceS(tempS, nextS);
            if (dist > -(safeRange + len) && dist < (safeRange + temp_vehicle.len)) {
                collisions.push_back(&temp_vehicle);
#ifdef DEBUG_OUT
                cout << "Collision course ahead - x " << temp_vehicle.x << " y " << temp_vehicle.y << " s " << temp_vehicle.s << " v " << MpS2MpH(temp_vehicle.v) 
                    << " vs: " << temp_vehicle.vs << " vd: " << temp_vehicle.vd << " Future dist " << Road::getCurrentRoad().normalizeS(dist) << endl;
#endif
            }
            else if (dist > -(safeRange + temp_vehicle.len) && dist < (safeRange + len)) {
                collisions.push_back(&temp_vehicle);
#ifdef DEBUG_OUT
                cout << "Collision course behind - x " << temp_vehicle.x << " y " << temp_vehicle.y << " s " << temp_vehicle.s << " v " << MpS2MpH(temp_vehicle.v) << " Future "
                        << Road::getCurrentRoad().normalizeS(dist) << endl;
#endif
            }
#ifdef DEBUG_OUT
            else {
                cout << "Not on clission course - s: " << tempS << " dist: " << dist << endl;
            }
#endif
        }
    }
    return collisions;
}

Vehicle* Vehicle::getVehicleBehindAt(vector<Vehicle> &fusion, int lane, double t) {
    double minDist = 10000000000;
    double nextS = sAfter(t);
#ifdef DEBUG_OUT
    cout << "Check vehicle on " << lane << " after " << t << " with s: " << nextS << endl;
#endif
    Vehicle *vehicle = NULL;
    double otherVehicleTime = Config::N * Config::dt;
    for (Vehicle &temp_vehicle: fusion) {
        if (onSameLane(Road::getCurrentRoad().laneToCenterD(lane), temp_vehicle.dAfter(otherVehicleTime), temp_vehicle)) {
            double tempS = temp_vehicle.sAfter(otherVehicleTime);
#ifdef DEBUG_OUT
            cout << "Check vehicle on same lane with s: " << tempS << endl;
#endif
            double dist = Road::getCurrentRoad().distanceS(tempS, nextS) - len; // take this's length into account
            if (dist < 0 && fabs(dist) < minDist) {
                minDist = fabs(dist);
                vehicle = &temp_vehicle;
            }
        }
    }
    
#ifdef DEBUG_OUT
    if (vehicle != NULL) {
        cout << "Found behind - x " << vehicle->x << " y " << vehicle->y << " s " << vehicle->s << " v " << MpS2MpH(vehicle->v) << " Future "
                << Road::getCurrentRoad().normalizeS(nextS - minDist) << endl;
    }
#endif
    return vehicle;
}

Vehicle* Vehicle::getVehicleAheadAt(vector<Vehicle> &fusion, int lane, double t) {
    double minDist = 10000000000;
    double nextS = sAfter(t);
#ifdef DEBUG_OUT
    cout << "Check vehicle on " << lane << " after " << t << " with s: " << nextS << endl;
#endif
    Vehicle *vehicle = NULL;
    double otherVehicleTime = t; // assume the sensor fusion has the prediction applied to the end of previous path point
    for (Vehicle &temp_vehicle: fusion) {
        if (onSameLane(Road::getCurrentRoad().laneToCenterD(lane), temp_vehicle.dAfter(otherVehicleTime), temp_vehicle)) {
            double tempS = temp_vehicle.sAfter(otherVehicleTime);
#ifdef DEBUG_OUT
            cout << "Check vehicle on lane with s: " << tempS << endl;
#endif
            double dist = Road::getCurrentRoad().distanceS(tempS, nextS) - Config::Lf * 1.5; // take the vehicle's length into account
            if (dist > 0 && fabs(dist) < minDist) {
                minDist = fabs(dist);
                vehicle = &temp_vehicle;
            }
        }
    }
#ifdef DEBUG_OUT
    if (vehicle != NULL) {
        cout << "Found ahead - x " << vehicle->x << " y " << vehicle->y << " s " << vehicle->s << " v " << MpS2MpH(vehicle->v) << " Future " << (nextS + minDist) << endl;
    }
#endif
    return vehicle;
}

bool Vehicle::onSameLane(const double my_d, const double another_d, const Vehicle &another) {
    double l = my_d - width * 0.5 - Config::minSafeGap;
    double r = l + width + 2 * Config::minSafeGap;
    double l2 = another_d - another.width * 0.5;
    double r2 = l2 + another.width;
    return r > l2 && l < r2;
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
