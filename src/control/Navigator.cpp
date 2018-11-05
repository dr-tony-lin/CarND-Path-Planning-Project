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
    }
    
    laneFusion = new vector<Vehicle*>[Road::current().numberOfLanes()];
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
        vector<double> fNormal = unitv(Road::current().distanceS(vehicle->s, last2[3]), vehicle->d - last2[4]);
        vehicle->vs = vehicle->v * fNormal[0];
        vehicle->vd = vehicle->v * fNormal[1];
        vehicle->as = vehicle->a * fNormal[0];
        vehicle->ad = vehicle->a * fNormal[1];
        for (unsigned i = 0; i < Config::N - previous_x.size() && predictions.size() > 0; i++) {
            predictions.pop_front();
        }
    }
}

vector<vector<double>> Navigator::navigate(vector<Vehicle> &vehicles) {
    if (currentTransition != NULL && currentTransition->target != NavigationState::KL) {
        if (//Road::current().distanceS(currentTransition->targetS, vehicle->s) < 0.1 ||
            fabs(vehicle->d - Road::current().laneToCenterD(currentTransition->immediateLane)) < 0.01) { // we have reached the goal
#ifdef INFO_OUT
        cout << "Transition ended: " << currentTransition->current << "  to " << currentTransition->target << " lane: " << currentTransition->immediateLane
            << " min speed ahead: " << currentTransition->limits.minSpeedAhead << " max speed behind:" << currentTransition->limits.maxSpeedBehind << " dist ahead: " << currentTransition->limits.distanceAhead
            << " dist behind: " << currentTransition->limits.distanceAhead << " current d: " << vehicle->d << endl;
#endif
            state = NavigationState::KL;
            delete currentTransition;
            currentTransition = NULL;
            lastLaneChangeTime = time_since_epoch();
        }
    }

    if (laneFusion != NULL) { // Clear previous fusion data
        for (int i = 0; i < Road::current().numberOfLanes(); i++) {
            laneFusion[i].clear();
        }
    }
    else {
        laneFusion = new vector<Vehicle*>[Road::current().numberOfLanes()];
    }

    this->vehicles.clear();

    // Find cars on the left and right lanes that are close enought to check
    for (Vehicle &v: vehicles) {
        if (fabs(Road::current().distanceS(v.s, vehicle->s)) < Config::maxDistance) { // consider car close enough to check
            this->vehicles.push_back(v);
        }
    }
    for (Vehicle &v: this->vehicles) {
        int vehicleLaneLeft = max(0, Road::current().dToLane(v.d - v.width * 0.5));
        int vehicleLaneRight = min(Road::current().dToLane(v.d + v.width * 0.5), Road::current().numberOfLanes());
        for (int lane = vehicleLaneLeft; lane <= vehicleLaneRight; lane++) { 
            orderedInsert(laneFusion[lane], &v);
        }
    }

    int currentLane = Road::current().dToLane(vehicle->d);

    int horizon = (Config::N - predictions.size());
    vector<StateTransition> transitions;
#ifdef DEBUG_OUT
        cout << "Creating transitions ..." << transitions.size() << endl;
#endif
    getLaneStateTransitions(transitions, currentLane, horizon * Config::dt);
    
    StateTransition *transition = NULL;
    StateTransition *kl = NULL;
    double minCost = 1e16;
    double klCost = 1e16;
#ifdef DEBUG_OUT
    cout << "Transitions generated: " << transitions.size() << endl;
#endif
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
        cout << "Transition " << it->current << "  to " << it->target << " lane: " << it->immediateLane
            << " min speed ahead: " << it->limits.minSpeedAhead << " max speed behind:" << it->limits.maxSpeedBehind << " dist ahead: " << it->limits.distanceAhead
            << " dist behind: " << it->limits.distanceBehind << " Cost: " << c << endl;
#endif
    }
    if (transition != kl && klCost - minCost< 0.05) { // the cost difference is too small, avoid prematurely change lane
        transition = kl; 
    }
    // Compute lane change distance if it is to take place
    double laneChangeDistance = Config::maxLaneChangeDistance;
    if (transition != NULL) {
        if (currentTransition != NULL) {
            laneChangeDistance = min(Config::maxLaneChangeDistance, 1.5 * min(currentTransition->limits.distanceAhead, transition->limits.distanceAhead));
        }
        else {
            laneChangeDistance = min(Config::maxLaneChangeDistance, transition->limits.distanceAhead);
        }

        currentTransition = new StateTransition(*transition);
        state = currentTransition->target;
        currentTransition->startS = vehicle->s;
        currentTransition->startD = vehicle->d;
        currentTransition->targetS = vehicle->s + laneChangeDistance;
        cout << "Pick transition " << transition->current << "  to " << transition->target << " to lane: " << transition->immediateLane
            << " from  lane: " << transition->sourceLane << " min speed ahead: " << transition->limits.minSpeedAhead << " max speed behind:" << transition->limits.maxSpeedBehind 
            << " dist ahead: " << transition->limits.distanceAhead << " dist behind: " << transition->limits.distanceBehind << " vehicle ahead: " << transition->nVehicleAhead
            << " vehicle behind: " << transition->nVehicleBehind << " lane change distance: " << laneChangeDistance << " Cost: " << minCost << endl;
    }

    return generateTrajectory(laneChangeDistance);
}

void Navigator::getLaneStateTransitions(vector<StateTransition> &transitions, int currentLane, double t) {
    double nextS = vehicle->sAfter(t);
    double nextV = vehicle->vAfter(t);

    StateTransition *kl = NULL;
    if (currentTransition == NULL || currentTransition->target == NavigationState::KL) { // update keep lane transition
        getLaneStateTransition(transitions, currentLane, currentLane, 0);
        kl = &transitions.front();
    }
    
    if (currentLane > 0 && (currentTransition == NULL ||
            (currentTransition->target != NavigationState::KL && currentTransition->targetLane == currentLane - 1) ||
            (currentTransition->target == NavigationState::KL && time_since_epoch() - lastLaneChangeTime > Config::minLaneChangeFreezeTime))) {
        getLaneStateTransition(transitions, currentLane - 1, currentLane, -1); // With this it is possible to change two lanes
        StateTransition &last = transitions.back();
        if (kl != NULL && kl != &last) {
            if (kl->target == NavigationState::KL) {
                last.previousLimits = kl->limits;
            }
            else {
                last.previousLimits = kl->previousLimits;
            }
        }
    }
    if (currentLane < Road::current().getNumberOfLanesAt(nextS) - 1 && (currentTransition == NULL ||
            (currentTransition->target != NavigationState::KL && currentTransition->targetLane == currentLane + 1) ||
            (currentTransition->target == NavigationState::KL && time_since_epoch() - lastLaneChangeTime > Config::minLaneChangeFreezeTime))) {
        getLaneStateTransition(transitions, currentLane + 1, currentLane, 1);
        StateTransition &last = transitions.back();
        if (kl != NULL && kl != &last) {
            if (kl->target == NavigationState::KL) {
                last.previousLimits = kl->limits;
            }
            else {
                last.previousLimits = kl->previousLimits;
            }
        }
    }
}

void Navigator::getLaneStateTransition(vector<StateTransition> &transitions, const int targetLane, const int currentLane, const int laneChange) {
    /**
     * Skip lane change preparation state for now, notheing to do in the simulator
     */ 
    vector<Vehicle *> &fusion = laneFusion[targetLane]; 
    if (fusion.size() == 0) { // no vehicle on the lane in the maximum horizon
        if ( currentTransition == NULL || (currentTransition->target == NavigationState::KL || 
                (currentTransition->targetLane != currentLane + laneChange && currentTransition->targetLane - currentTransition->sourceLane == laneChange))) {
            // No transition, or keep lane, or in transition but the target lane is different from the lane we are checking - can change two lanes but no bounce back
            NavigationState targetState;
            if (laneChange == -1) {
                targetState = NavigationState::LCL; // state == NavigationState::PLCL? NavigationState::LCL: NavigationState::PLCL;
            }
            else if (laneChange == 1) {
                targetState = NavigationState::LCR; // state == NavigationState::PLCR? NavigationState::LCR: NavigationState::PLCR;
            }
            else {
                targetState = state;
            }
            transitions.push_back(
                        StateTransition(state, 
                                        targetState,
                                        currentLane,
                                        currentLane + laneChange,
                                        Road::current().getSpeedLimit(),
                                        0,
                                        Config::maxDistance,
                                        -Config::maxDistance,
                                        0,
                                        0));
#ifdef DEBUG_OUT
            StateTransition t = transitions.back();
            cout << "Create no vehicle transition on " << currentLane << " from " << t.current << "  to " << t.target << " lane: " << t.immediateLane
                << " min speed ahead: " << t.limits.minSpeedAhead << " max speed behind:" << t.limits.maxSpeedBehind << " dist ahead: " << t.limits.distanceAhead
                << " dist behind: " << t.limits.distanceAhead << endl;
#endif
        }
    }
    else {
#ifdef DEBUG_OUT
        cout << "Generate transitions from " << currentLane << "  to " << (currentLane + laneChange) << " speed limit: " << Road::current().getSpeedLimit() << endl;
#endif
        double distanceAhead = 1000000;
        double distanceBehind = -1000000;
        double minSpeedAhead = Road::current().getSpeedLimit();
        double maxSpeedBehind = 0;
        double speedAhead = Road::current().getSpeedLimit();
        double speedBehind = 0;
        int nAhead = 0;
        int nBehind = 0;
        for (Vehicle *v: fusion) {
            if (v->distanceToTarget > 0) {
                if (v->distanceToTarget < distanceAhead) {
                    distanceAhead = v->distanceToTarget;
                    speedAhead = v->v;
                }
                minSpeedAhead = min(v->v, minSpeedAhead);
                nAhead++;
            }
            else {
                if (v->distanceToTarget > distanceBehind) {
                    distanceBehind = v->distanceToTarget;
                    speedBehind = v->v;
                }
                maxSpeedBehind = max(v->v, maxSpeedBehind);
                nBehind++;
            }
        }
#ifdef DEBUG_OUT
        cout << "Dist ahead: " << distanceAhead << " dist behind: " << distanceBehind << " min V ahead: " << minSpeedAhead 
            << " max V behind: " << maxSpeedBehind << " V ahead: " << speedAhead << " V behind: "  << speedBehind
            << " cars ahead: " << nAhead << " cars behind: " << nBehind << endl;
#endif
        if (laneChange == 0) { // Keep lane, we will always generate keep lane transition to update speed, and detect dangers
            transitions.push_back(
                StateTransition(state, 
                                state,
                                currentLane,
                                currentLane,
                                minSpeedAhead,
                                maxSpeedBehind,
                                distanceAhead,
                                distanceBehind,
                                nAhead,
                                nBehind));
#ifdef DEBUG_OUT
            StateTransition t = transitions.back();
            cout << "Create keep lane transition on " << currentLane << " from " << t.current << "  to " << t.target << " lane: " << t.immediateLane
                << " min speed ahead: " << t.limits.minSpeedAhead << " max speed behind:" << t.limits.maxSpeedBehind << " dist ahead: " << t.limits.distanceAhead
                << " dist behind: " << t.limits.distanceBehind << endl;
#endif
        }
        else { // we are seeking opportunity to run faster
            double minAheadDistance = Config::safeLaneChangeFrontDistance(fabs(minSpeedAhead - vehicle->v));
            double minBehindDistance = Config::safeLaneChangeBackDistance(fabs(maxSpeedBehind - vehicle->v));
#ifdef DEBUG_OUT
            cout << "Min dist ahead " << minAheadDistance << " min dist behind: " << minBehindDistance << " Dist ahead: " << distanceAhead << " dist behind: " << distanceBehind << endl;
#endif
            if (distanceAhead < minAheadDistance && -distanceBehind < minBehindDistance && minSpeedAhead < maxSpeedBehind) {
                if (state != NavigationState::KL) { // squeezed, we have to rollback if we are changing lane
                    NavigationState targetState;
                    if (laneChange == -1) {
                        targetState = NavigationState::KL; // state == NavigationState::PLCL? NavigationState::KL: NavigationState::PLCL;
                    }
                    else {
                        targetState = NavigationState::KL; // state == NavigationState::PLCR? NavigationState::KL: NavigationState::PLCR;
                    }
                    transitions.push_back( // basically, we are rolling back here, sould we also sort out the vehicle on the lane and process accordingly?
                                StateTransition(state, 
                                                targetState,
                                                currentLane,
                                                currentLane,
                                                minSpeedAhead,
                                                maxSpeedBehind,
                                                Config::maxDistance,
                                                -Config::maxDistance,
                                                nAhead,
                                                nBehind));
#ifdef DEBUG_OUT
                    StateTransition t = transitions.back();
                    cout << "Create rollback transition on " << currentLane << " from " << t.current << "  to " << t.target << " lane: " << t.immediateLane
                        << " min speed ahead: " << t.limits.minSpeedAhead << " max speed behind:" << t.limits.maxSpeedBehind << " dist ahead: " << t.limits.distanceAhead
                        << " dist behind: " << t.limits.distanceBehind << endl;
#endif
                }
#ifdef DEBUG_OUT
                else {
                    cout << "No transition minAheadDistance " << minAheadDistance << " distanceAhead: " << distanceAhead
                            << " minBehindDistance:  " << minBehindDistance << " distanceBehind: " << distanceBehind 
                            << " minSpeedAhead: " << minSpeedAhead << " Vehicle V: " << vehicle->v << endl;
                }
#endif
            }
            else if ( currentTransition != NULL && (currentTransition->target == NavigationState::KL || 
                (currentTransition->targetLane != currentLane + laneChange && currentTransition->targetLane - currentTransition->sourceLane == laneChange))) {
                NavigationState targetState;
                if (laneChange == -1) {
                    targetState = NavigationState::LCL; //state == NavigationState::PLCL? NavigationState::LCL: NavigationState::PLCL;
                }
                else {
                    targetState = NavigationState::LCR; //state == NavigationState::PLCR? NavigationState::LCR: NavigationState::PLCR;
                }
                if (distanceAhead < 1000000 - EPSILON) { // we have car in front, and distance is OK
                    if (distanceBehind > -1000000 + EPSILON) { // we have car behind, and distance is OK
                        // let the planner decide what to do if maxSpeedBehind < minSpeedAhead
                        transitions.push_back(
                                StateTransition(state, 
                                                targetState,
                                                currentLane,
                                                currentLane + laneChange,
                                                minSpeedAhead,
                                                maxSpeedBehind,
                                                distanceAhead,
                                                distanceBehind,
                                                nAhead,
                                                nBehind));
#ifdef DEBUG_OUT
                    StateTransition t = transitions.back();
                    cout << "Create front back transition on " << currentLane << " from " << t.current << "  to " << t.target << " lane: " << t.immediateLane
                        << " min speed ahead: " << t.limits.minSpeedAhead << " max speed behind:" << t.limits.maxSpeedBehind << " dist ahead: " << t.limits.distanceAhead
                        << " dist behind: " << t.limits.distanceBehind << endl;
#endif
                    }
                    else { // we don't have car behind
                        transitions.push_back(
                                StateTransition(state, 
                                                targetState,
                                                currentLane,
                                                currentLane + laneChange,
                                                minSpeedAhead,
                                                0,
                                                distanceAhead,
                                                -Config::maxDistance,
                                                nAhead,
                                                nBehind));
#ifdef DEBUG_OUT
                    StateTransition t = transitions.back();
                    cout << "Create front transition on " << currentLane << " from " << t.current << "  to " << t.target << " lane: " << t.immediateLane
                        << " min speed ahead: " << t.limits.minSpeedAhead << " max speed behind:" << t.limits.maxSpeedBehind << " dist ahead: " << t.limits.distanceAhead
                        << " dist behind: " << t.limits.distanceBehind << endl;
#endif
                    }
                }
                else if (distanceBehind > -1000000 + EPSILON) { // we have car behind, but no car in front
                    transitions.push_back(
                            StateTransition(state, 
                                            targetState,
                                            currentLane,
                                            currentLane + laneChange,
                                            Road::current().getSpeedLimit(),
                                            maxSpeedBehind,
                                            Config::maxDistance,
                                            distanceBehind,
                                            nAhead,
                                            nBehind));
#ifdef DEBUG_OUT
                    StateTransition t = transitions.back();
                    cout << "Create behind transition on " << currentLane << " from " << t.current << "  to " << t.target << " lane: " << t.immediateLane
                        << " min speed ahead: " << t.limits.minSpeedAhead << " max speed behind:" << t.limits.maxSpeedBehind << " dist ahead: " << t.limits.distanceAhead
                        << " dist behind: " << t.limits.distanceBehind << " " << distanceBehind << endl;
#endif
                }
            }
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
    cout <<  "Generate prediction from  s: " << vehicle->s << " d: " << vehicle->d << " v: " << vehicle->v << " horizon: " << horizon 
         << " to lane: " << toLane << " new d: " << newD << endl;
#endif
  vector<vector<double>> preds = vehicle->generatePredictions(vehicle->s, vehicle->d, vehicle->v, getAcceleration(*currentTransition), 
                                                                    newD, startS, startD, laneChangeDistance, horizon);
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
    if (transition.nVehicleAhead == 0) {
        return [](double t)->double {return Config::maxAcceleration;};
    }

    // Here is the easiest approach is just to estimate the acceleration and let the navigation to correct the speed as it goes
    double minDist = (transition.target == NavigationState::KL? Config::minSafeDistance: Config::minLaneChangeFrontDistance);
    double distance = transition.limits.distanceAhead - minDist;
    double vDiff = transition.limits.minSpeedAhead - vehicle->v;
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
    }
    else {
        v->distanceToTarget += vehicle->len;
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