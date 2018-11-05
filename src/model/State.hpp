#ifndef _MODEL_STATE_HPP
#define _MODEL_STATE_HPP
#include "../utils/Config.hpp"

enum NavigationState {
    KL = 1,
    LCL,
    LCR,
    PLCL,
    PLCR,
    COLLIIDED
};

struct Limits {
    double minSpeedAhead = 0;
    double maxSpeedBehind = 0;
    double distanceAhead = 0;
    double distanceBehind = 0;

    Limits() {}
    
    Limits(const double minVAhead, const double maxVBehind, const double dAhead, const double dBehind):
         minSpeedAhead(minVAhead), maxSpeedBehind(maxVBehind), distanceAhead(dAhead), distanceBehind(dBehind) {}

    Limits(const Limits &another) {
        minSpeedAhead = another.minSpeedAhead;
        maxSpeedBehind = another.maxSpeedBehind;
        distanceAhead = another.distanceAhead;
        distanceBehind = another.distanceBehind;
    }

    Limits& operator=(const Limits& another) {
        minSpeedAhead = another.minSpeedAhead;
        maxSpeedBehind = another.maxSpeedBehind;
        distanceAhead = another.distanceAhead;
        distanceBehind = another.distanceBehind;
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
    int nVehicleAhead;
    int nVehicleBehind;

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
        nVehicleAhead = another.nVehicleAhead;
        nVehicleBehind = another.nVehicleBehind;
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
        nVehicleAhead = another.nVehicleAhead;
        nVehicleBehind = another.nVehicleBehind;
    }

    StateTransition(const NavigationState from, const NavigationState to, const int source, const int immediate,
                    const double minVAhead, const double maxVBehind, const double dAhead, const double dBehind,
                    const int nVehAhead, const int nVehBehind): 
                            current(from), target(to), sourceLane(source), immediateLane(immediate), 
                            limits(minVAhead,maxVBehind,dAhead,dBehind), previousLimits(minVAhead,maxVBehind,dAhead,dBehind),
                            nVehicleAhead(nVehAhead), nVehicleBehind(nVehBehind), targetLane(Config::targetLane) {};
};
#endif