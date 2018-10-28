#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>
#include <iterator>
#include "../utils/utils.h"
#include "../model/Road.hpp"
#include "../model/Vehicle.hpp"
#include "Navigator.hpp"

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

vector<vector<double>> Navigator::navigate(vector<Vehicle> &fusion) {
    vector<vector<double>> trajectory;
    int horizon = (Config::N - predictions.size());
    int lane = Config::numberOfLanes; // on the side of the road
    double targetD, maxAccel;
    switch (state) {
      case NavigationState::KL:
      case NavigationState::PLCL:
      case NavigationState::PLCR: {
        lane = Road::getCurrentRoad().dToLane(vehicle->d);
        break;
      }
      case NavigationState::LCL: {
        double newD = vehicle->d - Config::laneWidth;
        lane = Road::getCurrentRoad().dToLane(newD);
        break;
      }
      case NavigationState::LCR: {
        double newD = vehicle->d + Config::laneWidth;
        lane = Road::getCurrentRoad().dToLane(newD);
        break;
      }
    }

    maxAccel = getMaxAcceleration(fusion, lane, horizon * Config::dt);
    targetD = Road::getCurrentRoad().laneToCenterD(lane);

    switch (state) {
      case NavigationState::KL:{
        if (vehicle->v < Road::getCurrentRoad().getSpeedLimit() * 0.9 && maxAccel < EPSILON) { // we are not fast enough

        }
        break;
      }
      case NavigationState::PLCL:
      case NavigationState::PLCR: {
          
        break;
      }
      case NavigationState::LCL:  {
        double newD = vehicle->d - Config::laneWidth;
        lane = Road::getCurrentRoad().dToLane(newD);
        break;
      }
      case NavigationState::LCR: {
        double newD = vehicle->d + Config::laneWidth;
        lane = Road::getCurrentRoad().dToLane(newD);
        break;
      }
    }

    return trajectory;
}

 vector<StateTransition> Navigator::getLaneChangeTransitions(vector<Vehicle> &fusion, int currentLane, double t) {
    double nextS = vehicle->sAfter(t);
    double nextV = vehicle->vAfter(t);
    vector<StateTransition> transitions;
    vector<Vehicle> vehiclesOnLeft;
    vector<Vehicle> vehiclesOnRight;
    vector<Vehicle> vehiclesOnLane;

    // Find cars on the left and right lanes that are close enought to check
    for (Vehicle &car: fusion) {
        int carLane = Road::getCurrentRoad().dToLane(car.d);
        if ((carLane <= currentLane + 1 && carLane >= currentLane - 1) && fabs(car.s - vehicle->s) < Config::maxDistance) { // consider car close enough to check
            if (carLane == currentLane + 1) { // om right lane
                vehiclesOnRight.push_back(car);
            }
            else if (carLane == currentLane - 1) { // on left lane
                vehiclesOnLeft.push_back(car);
            }
            else { // on current lane
                vehiclesOnLane.push_back(car);
            }
        }
    }

    if (currentLane > 0) {
        getLaneChangeTransition(transitions, vehiclesOnLeft, true);
    }
    if (currentLane < Road::getCurrentRoad().getNumberOfLanes(nextS) - 1) {
        getLaneChangeTransition(transitions, vehiclesOnRight, false);
    }

    return transitions;
}

void Navigator::getLaneChangeTransition(vector<StateTransition> &transitions, vector<Vehicle> &fusion, bool isLeft) {
    if (fusion.size() == 0) { // no vehicle on the lane in the maximum horizon, good to turn into
        NavigationState targetState;
        if (isLeft) {
            targetState = state == NavigationState::PLCL? NavigationState::LCL: NavigationState::PLCL;
        }
        else {
            targetState = state == NavigationState::PLCR? NavigationState::LCR: NavigationState::PLCR;
        }
        transitions.push_back(
                    StateTransition(state, 
                                    targetState,
                                    Road::getCurrentRoad().getSpeedLimit(),
                                    Road::getCurrentRoad().getSpeedLimit(),
                                    Config::maxDistance,
                                    Config::maxDistance));
    }
    else {
        double distanceAhead = 1000000;
        double distanceBehind = -1000000;
        double minSpeedAhead = 1000000;
        double maxSpeedBehind = -1000000;
        double speedAhead = 0;
        double speedBehind = 0;
        sortVehicle(fusion);
        for (auto car: fusion) {
            if (car.distanceToTarget > 0) {
                if (car.distanceToTarget < distanceAhead) {
                    distanceAhead = car.distanceToTarget;
                    speedAhead = car.v;
                }
                minSpeedAhead = min(car.v, minSpeedAhead);
            }
            else if (car.distanceToTarget < 0) {
                if (car.distanceToTarget > distanceBehind) {
                    distanceBehind = car.distanceToTarget;
                    speedBehind = car.v;
                }
                maxSpeedBehind = max(car.v, maxSpeedBehind);
            }
        }

        if (distanceAhead < Config::safeLaneChangeDistance(fabs(minSpeedAhead - vehicle->s)) || 
            distanceBehind < Config::safeLaneChangeDistance(fabs(maxSpeedBehind - vehicle->s)) ||
            minSpeedAhead < vehicle->v * 1.1) { // not good to change lane
            NavigationState targetState;
            if (isLeft) {
                targetState = state == NavigationState::PLCL? NavigationState::KL: NavigationState::PLCL;
            }
            else {
                targetState = state == NavigationState::PLCR? NavigationState::KL: NavigationState::PLCR;
            }
            transitions.push_back( // basically, we are rolling back here, sould we also sort out the vehicle on the lane and process accordingly?
                        StateTransition(state, 
                                        targetState,
                                        vehicle->v,
                                        vehicle->v,
                                        Config::maxDistance,
                                        Config::maxDistance));
        }
        else {
            NavigationState targetState;
            if (isLeft) {
                targetState = state == NavigationState::PLCL? NavigationState::LCL: NavigationState::PLCL;
            }
            else {
                targetState = state == NavigationState::PLCR? NavigationState::LCR: NavigationState::PLCR;
            }
            if (distanceAhead < 1000000 - EPSILON) { // we have car in front, and distance is OK
                if (distanceBehind > 1000000 + EPSILON) { // we have car behind, and distance is OK
                    // let the planner decide what to do if maxSpeedBehind < minSpeedAhead
                    transitions.push_back(
                            StateTransition(state, 
                                            targetState,
                                            minSpeedAhead,
                                            maxSpeedBehind,
                                            distanceAhead,
                                            distanceBehind));
                }
                else { // we don't have car behind
                    transitions.push_back(
                            StateTransition(state, 
                                            targetState,
                                            minSpeedAhead,
                                            minSpeedAhead,
                                            distanceAhead,
                                            Config::maxDistance));
                }
            }
            else if (distanceBehind > 1000000 + EPSILON) { // we have car behind, but no car in front
                transitions.push_back(
                        StateTransition(state, 
                                        targetState,
                                        Road::getCurrentRoad().getSpeedLimit(),
                                        Road::getCurrentRoad().getSpeedLimit(),
                                        Config::maxDistance,
                                        distanceBehind));
            }
        }
    }
}

vector<vector<double>> Navigator::generateTrajectory(vector<Vehicle> &fusion, int laneShift) {
  // TODO check if the lane shift is legal
  double newD = vehicle->d + laneShift * Config::laneWidth;
  int lane = Road::getCurrentRoad().dToLane(newD);
  int horizon = (Config::N - predictions.size());
  double maxAccel = getMaxAcceleration(fusion, lane, horizon * Config::dt);
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

double Navigator::getMaxAcceleration(vector<Vehicle> &fusion, int lane, double t) {
    vector<Vehicle *> onCollisionCourse = findVehicleOnCollisionCourse(fusion, lane, Config::N * Config::dt, 10);
    double newV = vehicle->vAfter(t); // the speed after t if we keep current accel
    double newS = vehicle->sAfter(t); // the position after t if we keep current speed
    if (state == NavigationState::COLLIIDED) { // we have collided with another vehicle, need a full stop full stop!
        return vehicle->v > 0? -vehicle->v: 0;
    }
    if (onCollisionCourse.size() == 0) {
        double accel = min(min((Road::getCurrentRoad().getSpeedLimit() - newV) / t, Config::maxJerk), vehicle->maxAcceleration);
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
        for (auto car: onCollisionCourse) {
            double collisionS = car->sAfter(t);
            double safeDistance = Config::safeDistance(newV - car->v);
            double distanceFront = Road::getCurrentRoad().distanceS(Road::getCurrentRoad().normalizeS(collisionS - Config::Lf * 1.5), newS);
            double distanceBack = Road::getCurrentRoad().distanceS(collisionS, Road::getCurrentRoad().normalizeS(newS - vehicle->len));
#ifdef DEBUG_OUT
            cout << "Check on collision course: s: " << car->s << " d: " << car->d << " v: " << MpS2MpH(car->v) << " vs: " << MpS2MpH(car->vs)
                    << " vd: " << MpS2MpH(car->vd) << " safe dist: " << safeDistance << " front dist: " << distanceFront << " back dist: " << distanceBack << endl;
#endif
            if (distanceFront > 0) {
                // use current s and v for distance
                double distance = Road::getCurrentRoad().distanceS(Road::getCurrentRoad().normalizeS(car->s - Config::Lf * 1.5 - safeDistance), vehicle->s); 
                double targetV = min(car->v + distance / t + 0.5 * (car->a - vehicle->a) * t, Road::getCurrentRoad().getSpeedLimit());
                double newA = (targetV - vehicle->v) / t;
                if (newA - vehicle->a > Config::maxJerk) {
                    newA = Config::maxJerk;
                }
                else if (newA - vehicle->a < -Config::maxJerk) {
                    newA = -Config::maxJerk;
                }
                if (targetV > Road::getCurrentRoad().getSpeedLimit() * 0.8) {
                    newA = newA * (Road::getCurrentRoad().getSpeedLimit() - targetV) / Road::getCurrentRoad().getSpeedLimit();
                }
                if (frontAccel > newA) {
                    frontAccel = newA;
                    bestFrontV = targetV;
                    bestSafeDistance = safeDistance;
                    vehicleV = car->v;
                    bestDistance = distance;
#ifdef DEBUG_OUT
                    cout << "New front, v: " << newV << " safe dist to keep: " << bestSafeDistance << " distance to travel: " << bestDistance
                            << " target v: " << bestFrontV << " accel: " << MpS2MpH(newA) << endl;
#endif
                }
            }
            else if (distanceBack < 0) { // Rear collission course
                // use current s and v for distance
                double distance = -Road::getCurrentRoad().distanceS(car->s, Road::getCurrentRoad().normalizeS(vehicle->s - vehicle->len - safeDistance)); 
                double minV = min(car->v + distance / t + 0.5 * (car->a - vehicle->a) * t, Road::getCurrentRoad().getSpeedLimit());
                double minA = (minV - vehicle->v) / t * 1.3; // 30% over what we need to avoid back collision
                if (minA - vehicle->a > Config::maxJerk) {
                    minA = Config::maxJerk;
                }
                else if (minA - vehicle->a < -Config::maxJerk) {
                    minA = -Config::maxJerk;
                }
                if (minV > Road::getCurrentRoad().getSpeedLimit() * 0.8) {
                    minA = minA * (Road::getCurrentRoad().getSpeedLimit() * 0.98 - minV) / Road::getCurrentRoad().getSpeedLimit();
                }
                if (backAccel < minA) {
                    backAccel = minA;
                    bestBackV = minV;
                    bestSafeDistance = safeDistance;
                    vehicleV = car->v;
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
        return result;
    }
}

vector<Vehicle*> Navigator::findVehicleOnCollisionCourse(vector<Vehicle> &fusion, int lane, double t, double safeRange) {
    double nextS = vehicle->sAfter(t);
    vector<Vehicle*> collisions;
#ifdef DEBUG_OUT
    cout << "Check vehicle on " << lane << " after " << t << " with s: " << nextS << endl;
#endif
    double otherVehicleTime = Config::N * Config::dt;
    for (Vehicle &car: fusion) {
#ifdef VERBOSE_OUT
        cout << "Check vehicle - s " << car.s << " v " << MpS2MpH(car.v) << " d: " << car.d << " vs: " << car.vs << " vd: " << car.vd
                << " furute: s: " << car.sAfter(otherVehicleTime) << " furute: d: " << car.dAfter(otherVehicleTime) << endl;
#endif
        // Assume the vehicle can not cross multiple lanes in the horizon, should be fine unless the horizon is too big
        if (vehicle->onSameLane(Road::getCurrentRoad().laneToCenterD(lane), car.dAfter(otherVehicleTime), car) ||
            vehicle->onSameLane(vehicle->d, car.d, car)) { // accept the situations that other vehicle may or may not change lane
            if (vehicle->hasCollision(car)) {
                collisions.push_back(&car);
                state = NavigationState::COLLIIDED;
#ifdef DEBUG_OUT
                cout << "Collision detected - x " << car.x << " y " << car.y << " s " << car.s << " d: " << car.d << " v " << MpS2MpH(car.v) 
                    << " vs: " << car.vs << " vd: " << car.vd << endl;
#endif
                continue;
            }
            double tempS = car.sAfter(otherVehicleTime);
            double dist = Road::getCurrentRoad().distanceS(tempS, nextS);
            if (dist > -(safeRange + vehicle->len) && dist < (safeRange + car.len)) {
                collisions.push_back(&car);
#ifdef DEBUG_OUT
                cout << "Collision course ahead - x " << car.x << " y " << car.y << " s " << car.s << " d: " << car.d << " v " << MpS2MpH(car.v) 
                    << " vs: " << car.vs << " vd: " << car.vd << " Future dist " << Road::getCurrentRoad().normalizeS(dist) << endl;
#endif
            }
            else if (dist > -(safeRange + car.len) && dist < (safeRange + vehicle->len)) {
                collisions.push_back(&car);
#ifdef DEBUG_OUT
                cout << "Collision course behind - x " << car.x << " y " << car.y << " s " << car.s << " d: " << car.d << " v " << MpS2MpH(car.v) << " Future "
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

Vehicle* Navigator::getVehicleBehindAt(vector<Vehicle> &fusion, int lane, double t) {
    double minDist = 10000000000;
    double nextS = vehicle->sAfter(t);
#ifdef DEBUG_OUT
    cout << "Check vehicle on " << lane << " after " << t << " with s: " << nextS << endl;
#endif
    Vehicle *vehicle = NULL;
    double otherVehicleTime = Config::N * Config::dt;
    for (Vehicle &car: fusion) {
        if (vehicle->onSameLane(Road::getCurrentRoad().laneToCenterD(lane), car.dAfter(otherVehicleTime), car)) {
            double tempS = car.sAfter(otherVehicleTime);
#ifdef DEBUG_OUT
            cout << "Check vehicle on same lane with s: " << tempS << endl;
#endif
            double dist = Road::getCurrentRoad().distanceS(tempS, nextS) - vehicle->len; // take this's length into account
            if (dist < 0 && fabs(dist) < minDist) {
                minDist = fabs(dist);
                vehicle = &car;
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

Vehicle* Navigator::getVehicleAheadAt(vector<Vehicle> &fusion, int lane, double t) {
    double minDist = 10000000000;
    double nextS = vehicle->sAfter(t);
#ifdef DEBUG_OUT
    cout << "Check vehicle on " << lane << " after " << t << " with s: " << nextS << endl;
#endif
    Vehicle *vehicle = NULL;
    double otherVehicleTime = t; // assume the sensor fusion has the prediction applied to the end of previous path point
    for (Vehicle &car: fusion) {
        if (vehicle->onSameLane(Road::getCurrentRoad().laneToCenterD(lane), car.dAfter(otherVehicleTime), car)) {
            double tempS = car.sAfter(otherVehicleTime);
#ifdef DEBUG_OUT
            cout << "Check vehicle on lane with s: " << tempS << endl;
#endif
            double dist = Road::getCurrentRoad().distanceS(tempS, nextS) - Config::Lf * 1.5; // take the vehicle's length into account
            if (dist > 0 && fabs(dist) < minDist) {
                minDist = fabs(dist);
                vehicle = &car;
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

void Navigator::sortVehicle(vector<Vehicle> vehicles) {
    for (auto car: vehicles) {
        car.distanceToTarget = Road::getCurrentRoad().distanceS(car.s, vehicle->s);
    }
    sort(vehicles.begin(), vehicles.end(), [](Vehicle &l, Vehicle &r) {
        return l.distanceToTarget > r.distanceToTarget;
    });
}