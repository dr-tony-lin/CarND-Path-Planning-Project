#ifndef _MODEL_STATE_HPP
#define _MODEL_STATE_HPP
#include "../utils/Config.hpp"

enum NavigationState {
    KL = 1,
    PLCL,
    PLCR,
    LCL,
    LCR,
    COLLIIDED
};

struct Limits {
    double minSpeedAhead = 0;
    double maxSpeedBehind = 0;
    double distanceAhead = 0;
    double distanceBehind = 0;
    double cost = 0;
    int nVehicleAhead;
    int nVehicleBehind;

    Limits() {}
    
    Limits(const double minVAhead, const double maxVBehind, const double dAhead, const double dBehind,
            const int nVehAhead, const int nVehBehind): minSpeedAhead(minVAhead), maxSpeedBehind(maxVBehind),
            distanceAhead(dAhead), distanceBehind(dBehind), nVehicleAhead(nVehAhead), nVehicleBehind(nVehBehind) {}

    Limits(const Limits &another) {
        minSpeedAhead = another.minSpeedAhead;
        maxSpeedBehind = another.maxSpeedBehind;
        distanceAhead = another.distanceAhead;
        distanceBehind = another.distanceBehind;
        nVehicleAhead = another.nVehicleAhead;
        nVehicleBehind = another.nVehicleBehind;
        cost = another.cost;
    }

    Limits& operator=(const Limits& another) {
        minSpeedAhead = another.minSpeedAhead;
        maxSpeedBehind = another.maxSpeedBehind;
        distanceAhead = another.distanceAhead;
        distanceBehind = another.distanceBehind;
        nVehicleAhead = another.nVehicleAhead;
        nVehicleBehind = another.nVehicleBehind;
        cost = another.cost;
    }
};

struct StateTransition {
    NavigationState current;
    NavigationState target;
    int sourceLane;
    int targetLane;
    int immediateLane;
    double targetS = 0;
    double targetV = 0;
    double startS = 0;
    double startD = 0;
    Limits limits;
    Limits previousLimits;

    StateTransition() {
    }

    StateTransition(const StateTransition &another) {
        current = another.current;
        target = another.target;
        sourceLane = another.sourceLane;
        immediateLane = another.immediateLane;
        targetLane = another.targetLane;
        targetS = another.targetS;
        startS = another.startS;
        startD = another.startD;
        limits = another.limits;
        previousLimits = another.previousLimits;
    }

    StateTransition& operator=(const StateTransition &another) {
        current = another.current;
        target = another.target;
        sourceLane = another.sourceLane;
        immediateLane = another.immediateLane;
        targetLane = another.targetLane;
        targetS = another.targetS;
        startS = another.startS;
        startD = another.startD;
        limits = another.limits;
        previousLimits = another.previousLimits;
    }

    StateTransition(const NavigationState from, const NavigationState to, const int source, const int immediate,
                    const double minVAhead, const double maxVBehind, const double dAhead, const double dBehind,
                    const int nVehAhead, const int nVehBehind): 
                            current(from), target(to), sourceLane(source), immediateLane(immediate), 
                            limits(minVAhead, maxVBehind, dAhead, dBehind, nVehAhead, nVehBehind), 
                            previousLimits(minVAhead, maxVBehind, dAhead, dBehind, nVehAhead, nVehBehind),
                            targetLane(Config::targetLane) {};
    
    StateTransition(const NavigationState from, const NavigationState to, const int source, const int immediate,
                    const Limits curLimits): 
                            current(from), target(to), sourceLane(source), immediateLane(immediate), 
                            limits(curLimits), previousLimits(curLimits),
                            targetLane(Config::targetLane) {};

    
    StateTransition(const NavigationState from, const NavigationState to, const int source, const int immediate,
                    const Limits curLimits, const Limits prevLimits): 
                            current(from), target(to), sourceLane(source), immediateLane(immediate), 
                            limits(curLimits), previousLimits(prevLimits),
                            targetLane(Config::targetLane) {};
};
#endif