#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>
#include <iterator>
#include <chrono>
#include "../utils/utils.h"
#include "../model/Road.hpp"
#include "../model/Vehicle.hpp"
#include "Navigator.hpp"

using namespace std;

long long time_since_epoch() {
    auto now = chrono::system_clock::now().time_since_epoch();
    return chrono::duration_cast<std::chrono::seconds>(now).count();
}

void Navigator::initializeVehicle(const double x, const double y, const double s, const double d,
                        const double yaw, const double v, const double LF, const double maxAccel) {              
    if (vehicle != NULL) {
        delete vehicle;
    }

    vehicle = new Vehicle(x, y, s, d, yaw, v);
    vehicle->setLength(LF);
    vehicle->maxAcceleration = maxAccel;
    predictions.clear();
    lastLaneChangeTime = 0;
    vehicles.clear();
    if (laneFusion != NULL) {
        for (int i = 0; i < Road::current().numberOfLanes(); i++) {
            laneFusion[i].clear();
        }

        delete laneFusion;
        delete laneLimits;
    }
    
    laneFusion = new vector<Vehicle*>[Road::current().numberOfLanes()];
    laneLimits  = new Limits[Road::current().numberOfLanes()];
}

void Navigator::update(vector<Vehicle> &vehicles, const vector<double>& previous_x, const vector<double>& previous_y) {
    if (predictions.size() > 0) {
        vehicle->x = predictions.back()[1];
        vehicle->y = predictions.back()[2];
        vehicle->s = predictions.back()[3];
        vehicle->d = predictions.back()[4];
        vehicle->v = predictions.back()[5];
        vehicle->a = predictions.back()[6];
        vector<double> last2 = predictions[predictions.size() - 2];
        vehicle->dx = vehicle->x - last2[1];
        vehicle->dy = vehicle->y - last2[2];
        vector<double> fNormal = unitv(Road::current().distanceS(vehicle->s, last2[3]), vehicle->d - last2[4]);
        vehicle->vs = vehicle->v * fNormal[0];
        vehicle->vd = vehicle->v * fNormal[1];
        vehicle->as = vehicle->a * fNormal[0];
        vehicle->ad = vehicle->a * fNormal[1];
        for (unsigned i = 0; i < Config::N - previous_x.size() && predictions.size() > 0; i++) {
            predictions.pop_front();
        }
    }
    if (laneFusion != NULL) { // Clear previous fusion data
        for (int i = 0; i < Road::current().numberOfLanes(); i++) {
            laneFusion[i].clear();
        }
    }
    else {
        laneFusion = new vector<Vehicle*>[Road::current().numberOfLanes()];
        laneLimits = new Limits[Road::current().numberOfLanes()];
    }

    this->vehicles.clear();

    // Find cars on the left and right lanes that are close enought to check
    for (Vehicle &v: vehicles) {
        if (fabs(Road::current().distanceS(v.s, vehicle->s)) < Config::maxDistance) { // consider car close enough to check
            this->vehicles.push_back(v);
        }
    }

    for (Vehicle &v: this->vehicles) {
        int vehicleLaneLeft = Road::current().dToLane(v.d - v.width * 0.5);
        int vehicleLaneRight = Road::current().dToLane(v.d + v.width * 0.5);
        for (int lane = vehicleLaneLeft; lane <= vehicleLaneRight; lane++) { 
            orderedInsert(laneFusion[lane], &v);
        }
    }

    for (int i = 0; i < Road::current().numberOfLanes(); i++) {
        getLaneLimits(laneFusion[i], laneLimits[i]);
    }
}

vector<vector<double>> Navigator::navigate() {
    if (currentTransition != NULL && currentTransition->target != NavigationState::KL) {
        if (//Road::current().distanceS(currentTransition->targetS, vehicle->s) < 0.1 ||
            fabs(vehicle->d - Road::current().laneToCenterD(currentTransition->immediateLane)) < 0.01) { // we have reached the goal
#ifdef INFO_OUT
        cout << "Transition ended: " << currentTransition->current << "  to " << currentTransition->target << " lane: " << currentTransition->immediateLane
            << " min speed ahead: " << currentTransition->limits.minSpeedAhead << " max speed behind:" << currentTransition->limits.maxSpeedBehind 
            << " dist ahead: " << currentTransition->limits.distanceAhead << " dist behind: " << currentTransition->limits.distanceAhead 
            << " current d: " << vehicle->d << endl;
#endif
            delete currentTransition;
            currentTransition = NULL;
            state = NavigationState::KL;
            lastLaneChangeTime = time_since_epoch();
        }
    }

    vector<StateTransition> transitions;
    int horizon = Config::N - predictions.size();
    getLaneStateTransitions(transitions, horizon * Config::dt);
    
    StateTransition *transition = NULL;
    StateTransition *kl = NULL;
    double minCost = 1e16;
    double klCost = 1e16;

    for (auto it = transitions.begin(); it != transitions.end(); it++) {
        double c = cost(*vehicle, *it);
        if (it->target == NavigationState::KL) {
            klCost = c;
            kl = &*it;
        }
        if (c < minCost) {
            minCost = c;
            transition = &*it;
        }
#ifdef DEBUG_OUT
        StateTransition &t = *it;
        cout << "Transition " << it->current << "  to " << it->target << " from lane: " << it->sourceLane << " to lane " << it->immediateLane
            << " min speed ahead: " << it->limits.minSpeedAhead << " max speed behind:" << it->limits.maxSpeedBehind << " dist ahead: " << it->limits.distanceAhead
            << " dist behind: " << it->limits.distanceBehind << " Cost: " << c << endl;
#endif
    }

    if (transition != kl && klCost - minCost< 0.05) { // the cost difference is too small, avoid prematurely lane change
        transition = kl; 
    }

    // Compute lane change distance if it is to take place
    double laneChangeDistance = Config::maxLaneChangeDistance;
    if (transition != NULL) {
        if (currentTransition != NULL) {
            laneChangeDistance = min(Config::maxLaneChangeDistance, min(transition->previousLimits.distanceAhead, transition->limits.distanceAhead));
            delete currentTransition;
        }
        else {
            laneChangeDistance = min(Config::maxLaneChangeDistance, transition->limits.distanceAhead);
        }
        
        Limits &sourceLimit = laneLimits[transition->sourceLane];
        laneChangeDistance += min(min(transition->limits.minSpeedAhead, transition->limits.maxSpeedBehind), 
                                  sourceLimit.minSpeedAhead) * Config::minLaneChangeTime;
        currentTransition = new StateTransition(*transition);
        currentTransition->targetS = vehicle->s + laneChangeDistance;
        cout << "Pick transition " << transition->current << "  to " << transition->target << " from lane: " << transition->sourceLane
            << " to  lane: " << transition->immediateLane << " min speed ahead: " << transition->limits.minSpeedAhead << " max speed behind:" << transition->limits.maxSpeedBehind 
            << " dist ahead: " << transition->limits.distanceAhead << " dist behind: " << transition->limits.distanceBehind << " vehicle ahead: " << transition->limits.nVehicleAhead
            << " vehicle behind: " << transition->limits.nVehicleBehind << " lane change distance: " << laneChangeDistance << " Cost: " << minCost << endl;
    }

    return generateTrajectory(laneChangeDistance);
}

void Navigator::getLaneStateTransitions(vector<StateTransition> &transitions, double t) {
#ifdef DEBUG_OUT
        cout << "Creating transitions left: " << vehicle->laneLeft << " right: " << vehicle->laneRight << endl;
#endif
    double nextS = vehicle->sAfter(t);
    double nextV = vehicle->vAfter(t);
    double currentLane = Road::current().dToLane(vehicle->d);
    if (currentTransition == NULL || currentTransition->target == NavigationState::KL) { // update keep lane transition
        StateTransition transition = getStateTransitionToLane(currentLane, currentLane);
        transition.startS = vehicle->s;
        transition.startD = vehicle->d;
        transitions.push_back(transition);
        if (currentLane > 0) {
            StateTransition left = getStateTransitionToLane(currentLane, currentLane - 1);
            transitions.push_back(left);
        }
        if (currentLane < Road::current().getNumberOfLanesAt(vehicle->s) - 1) {
            StateTransition right = getStateTransitionToLane(currentLane, currentLane + 1);
            transitions.push_back(right);
        }
    }
    else if (currentTransition != NULL && currentTransition->target != NavigationState::KL) { // Lane change
        StateTransition *rollback = updateLaneStateTransition(*currentTransition); // update current transition
        if (rollback != NULL) {
            transitions.push_back(*rollback);
            delete rollback;
        }
        // if (vehicle->laneLeft == currentTransition->immediateLane && vehicle->laneLeft == vehicle->laneRight) { 
        //     // we are on the target lane, check if we can change two lanes
        //     if (currentTransition->target == NavigationState::LCL && currentTransition->immediateLane > 0) {
        //         StateTransition transition = getStateTransitionToLane(currentTransition->immediateLane, currentTransition->immediateLane - 1);
        //         transition.startS = vehicle->s;
        //         transition.startD = vehicle->d;
        //         transitions.push_back(transition);
        //     }
        //     if (currentTransition->target == NavigationState::LCR && currentTransition->immediateLane < Road::current().numberOfLanes() - 1) {
        //         StateTransition transition = getStateTransitionToLane(currentTransition->immediateLane, currentTransition->immediateLane + 1);
        //         transition.startS = vehicle->s;
        //         transition.startD = vehicle->d;
        //         transitions.push_back(transition);
        //     }
        // }
        transitions.push_back(*currentTransition);
    }
}

StateTransition *Navigator::updateLaneStateTransition(StateTransition &transition) {
    /**
     * Skip lane change preparation state for now, notheing to do in the simulator
     */ 
    vector<Vehicle *> &sourceFusion = laneFusion[transition.sourceLane];
    vector<Vehicle *> &targetFusion = laneFusion[transition.immediateLane];
    transition.limits = laneLimits[transition.immediateLane];
    transition.previousLimits = laneLimits[transition.sourceLane];

    double minAheadDistance = Config::safeLaneChangeFrontDistance(fabs(transition.limits.minSpeedAhead - vehicle->v));
    double minBehindDistance = Config::safeLaneChangeBackDistance(fabs(transition.limits.maxSpeedBehind - vehicle->v));
    if (transition.limits.distanceAhead < minAheadDistance && -transition.limits.distanceBehind < minBehindDistance
         && transition.limits.minSpeedAhead < transition.limits.maxSpeedBehind) {
        if (transition.target != NavigationState::KL) { // squeezed, we have to rollback if we are changing lane
            StateTransition *rollback = new StateTransition(transition);
            rollback->target = NavigationState::KL;
            rollback->limits = transition.previousLimits;
            rollback->previousLimits = transition.limits;
            rollback->startS = vehicle->s;
            rollback->startD = vehicle->d;
            return rollback;
        }
    }

    return NULL;
}

StateTransition Navigator::getStateTransitionToLane(const int fromLane, const int toLane) {
    /**
     * Skip lane change preparation state for now
     */
    vector<Vehicle *> &sourceFusion = laneFusion[fromLane];
    vector<Vehicle *> &targetFusion = laneFusion[toLane];
    NavigationState targetState;
    if (fromLane == toLane) { // keep lane
        targetState = NavigationState::KL;
    }
    else if (fromLane > toLane) {
        targetState = NavigationState::LCL; // center lane is not yet cross treat as LCL
    }
    else if (fromLane < toLane) {
        targetState = NavigationState::LCR; // center lane is not yet cross, treat as LCR
    }

    StateTransition transition = StateTransition(state, 
                            targetState,
                            fromLane,
                            toLane,
                            laneLimits[toLane],
                            laneLimits[fromLane]);
    transition.startS = vehicle->s;
    transition.startD = vehicle->d;
    return transition;
}

void Navigator::getLaneLimits(vector<Vehicle *> &fusion, Limits &limits) {
    limits.distanceAhead = Config::maxDistance;
    limits.distanceBehind = -Config::maxDistance;
    limits.minSpeedAhead = Road::current().getSpeedLimit();
    limits.maxSpeedBehind = 0;
    limits.nVehicleAhead = 0;
    limits.nVehicleBehind = 0;
    for (Vehicle *v: fusion) {
        if (v->distanceToTarget >= 0) {
            limits.distanceAhead = min(limits.distanceAhead, v->distanceToTarget);
            limits.minSpeedAhead = min(limits.minSpeedAhead, v->v);
            limits.nVehicleAhead++;
        }
        else {
            limits.distanceBehind = max(limits.distanceBehind, v->distanceToTarget), 
            limits.maxSpeedBehind = max(limits.maxSpeedBehind, v->v);
            limits.nVehicleBehind++;
        }
    }
}

vector<vector<double>> Navigator::generateTrajectory(const double laneChangeDistance) {
  int toLane;
  int fromLane;
  double startS, startD;
  if (currentTransition == NULL) {
      toLane = fromLane = Road::current().dToLane(vehicle->d);
      startS = vehicle->s;
      startD = vehicle->d;
  }
  else {
      fromLane = currentTransition->sourceLane;
      toLane = currentTransition->immediateLane;
      startS = currentTransition->startS;
      startD = currentTransition->startD;
  }

  double newD = Road::current().laneToCenterD(toLane);
  int horizon = (Config::N - predictions.size());
#ifdef DEBUG_OUT
    cout <<  "Generate prediction from  s: " << vehicle->s << " d: " << vehicle->d << " v: " << vehicle->v
            << " horizon: " << horizon << " to lane: " << toLane << " new d: " << newD  << " start d: " << startD
            << " change distance: " << laneChangeDistance << " start S: " << startS  << endl;
#endif
  vector<vector<double>> preds = vehicle->generatePredictions(vehicle->s, vehicle->d, vehicle->v, 
                                getAcceleration(*currentTransition), newD, startS, startD, laneChangeDistance, horizon);
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
            << " d: " << predictions[i][4] << " v: " << predictions[i][5] << endl;
    }
#endif

  return {X, Y};
} 

std::function<double (double)> Navigator::getAcceleration(StateTransition &transition) {
    if (transition.limits.nVehicleAhead == 0) {
        return [](double t)->double {return Config::maxAcceleration;};
    }

    double minSpeedAhead = transition.limits.minSpeedAhead;
    double minDistAhead = transition.limits.distanceAhead;
    double minDist = Config::minSafeDistance;
    if (transition.target != NavigationState::KL && (vehicle->laneLeft == transition.sourceLane ||
        vehicle->laneRight == transition.sourceLane)) {
        minDist = Config::minLaneChangeFrontDistance;
        if (laneLimits[transition.sourceLane].distanceAhead / minSpeedAhead < Config::minLaneChangeTime) {
            minSpeedAhead = min(minSpeedAhead, laneLimits[transition.sourceLane].minSpeedAhead);
            minDistAhead = min(minDistAhead, laneLimits[transition.sourceLane].distanceAhead);
        }
    }

    // Here is the easiest approach is just to estimate the acceleration and let the navigation to correct the speed as it goes
    double distance = minDistAhead - minDist;
    double vDiff = minSpeedAhead - vehicle->v;
    double acel = vDiff + distance / 4.0;
    if (acel > 0) {
        acel = min(acel, Config::maxAcceleration);
    }
    else {
        acel = max(acel, Config::maxDeceleration);
    }
    return [acel](double t)->double {return acel;};
}

vector<Vehicle*> Navigator::findVehicleOnCollisionCourse(int lane, double t, double safeRange) {
    vector<Vehicle *> &fusion = laneFusion[lane];
    double nextS = vehicle->sAfter(t);
    vector<Vehicle*> collisions;
#ifdef DEBUG_OUT
    cout << "Check vehicle on " << lane << " after " << t << " with s: " << nextS << endl;
#endif
    double otherVehicleTime = Config::N * Config::dt;
    for (Vehicle *v: fusion) {
#ifdef VERBOSE_OUT
        cout << "Check vehicle - s " << v->s << " v " << v->v << " d: " << v->d << " vs: " << v->vs << " vd: " << v->vd
                << " furute: s: " << v->sAfter(otherVehicleTime) << " furute: d: " << v->dAfter(otherVehicleTime) << endl;
#endif
        // Assume the vehicle can not cross multiple lanes in the horizon, should be fine unless the horizon is too big
        if (vehicle->onSameLane(vehicle->dAfter(t), v->dAfter(otherVehicleTime), *v) ||
            vehicle->onSameLane(vehicle->d, v->d, *v)) { // accept the situations that other vehicle may or may not change lane
            Vehicle temp = *vehicle;
            if (predictions.size() > 0) {
                temp.s = predictions[0][3]; // use the current position to avoid early death
            }
            if (vehicle->hasCollision(*v) && temp.hasCollision(*v)) {
                collisions.push_back(v);
                state = NavigationState::COLLIIDED;
#ifdef ERROR_OUT
                cout << "Collision detected - x " << v->x << " y " << v->y << " s: " << v->s << " d: " << v->d << " v: " << v->v 
                    << " vs: " << v->vs << " vd: " << v->vd << " vehicle s: " << vehicle->s << ", " << temp.s << " vehicle d: " << vehicle->d
                    << " vehicle V: " << vehicle->v << " length: " << max(vehicle->len, v->len) << endl;
#endif
                return collisions; // no need to continue
            }
            double tempS = v->sAfter(otherVehicleTime);
            double dist = Road::current().distanceS(tempS, nextS);
            if (dist > -(safeRange + vehicle->len) && dist < (safeRange + v->len)) {
                collisions.push_back(v);
#ifdef DEBUG_OUT
                cout << "Collision course ahead - x " << v->x << " y " << v->y << " s " << v->s << " d: " << v->d << " v " << v->v
                    << " vs: " << v->vs << " vd: " << v->vd << " Future dist " << dist << endl;
#endif
            }
            else if (dist > -(safeRange + v->len) && dist < (safeRange + vehicle->len)) {
                collisions.push_back(v);
#ifdef DEBUG_OUT
                cout << "Collision course behind - x " << v->x << " y " << v->y << " s " << v->s << " d: " << v->d << " v " << v->v << " Future "
                        << dist << endl;
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

Vehicle* Navigator::getVehicleBehindAt(int lane, double t) {
    vector<Vehicle *> &fusion = laneFusion[lane];
    double minDist = 10000000000;
    double nextS = vehicle->sAfter(t);
#ifdef DEBUG_OUT
    cout << "Check vehicle on " << lane << " after " << t << " with s: " << nextS << endl;
#endif
    Vehicle *vehicle = NULL;
    double otherVehicleTime = Config::N * Config::dt;
    for (Vehicle *v: fusion) {
        if (vehicle->onSameLane(Road::current().laneToCenterD(lane), v->dAfter(otherVehicleTime), *v)) {
            double tempS = v->sAfter(otherVehicleTime);
#ifdef DEBUG_OUT
            cout << "Check vehicle on same lane with s: " << tempS << endl;
#endif
            double dist = Road::current().distanceS(tempS, nextS) - vehicle->len; // take this's length into account
            if (dist < 0 && fabs(dist) < minDist) {
                minDist = fabs(dist);
                vehicle = v;
            }
        }
    }
    
#ifdef DEBUG_OUT
    if (vehicle != NULL) {
        cout << "Found behind - x " << vehicle->x << " y " << vehicle->y << " s " << vehicle->s << " v " << vehicle->v << " Future "
                << Road::current().normalizeS(nextS - minDist) << endl;
    }
#endif
    return vehicle;
}

Vehicle* Navigator::getVehicleAheadAt(int lane, double t) {
    vector<Vehicle *> &fusion = laneFusion[lane];
    double minDist = 10000000000;
    double nextS = vehicle->sAfter(t);
#ifdef DEBUG_OUT
    cout << "Check vehicle on " << lane << " after " << t << " with s: " << nextS << endl;
#endif
    Vehicle *vehicle = NULL;
    double otherVehicleTime = t; // assume the sensor fusion has the prediction applied to the end of previous path point
    for (Vehicle *v: fusion) {
        if (vehicle->onSameLane(Road::current().laneToCenterD(lane), v->dAfter(otherVehicleTime), *v)) {
            double tempS = v->sAfter(otherVehicleTime);
#ifdef DEBUG_OUT
            cout << "Check vehicle on lane with s: " << tempS << endl;
#endif
            double dist = Road::current().distanceS(tempS, nextS) - Config::Lf * 1.5; // take the vehicle's length into account
            if (dist > 0 && fabs(dist) < minDist) {
                minDist = fabs(dist);
                vehicle = v;
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

void Navigator::orderedInsert(vector<Vehicle*> &vehicles, Vehicle *v) {
    v->distanceToTarget = Road::current().distanceS(v->s, vehicle->s);
    if (v->distanceToTarget >= 0) {
        v->distanceToTarget -= v->len;
        if (v->distanceToTarget < 0) {
            v->distanceToTarget = 0;
        }
    }
    else {
        v->distanceToTarget += vehicle->len;
        if (v->distanceToTarget > 0) {
            v->distanceToTarget = 0;
        }
    }
    if (vehicles.size() == 0) {
        vehicles.push_back(v);
    }
    else {
        for (unsigned i = 0; i < vehicles.size(); i++) {
            if (vehicles[i]->distanceToTarget < v->distanceToTarget) {
                vehicles.insert(vehicles.begin() + i, v);
                return;
            }
        }
        vehicles.push_back(v);
    }
}