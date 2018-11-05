#include "Cost.hpp"

using namespace std;

class SafetyCost: public Cost {
public:
    SafetyCost() {};
    double operator()(Vehicle &vehicle, StateTransition &transition) const {
        double tf = (vehicle.v - transition.limits.minSpeedAhead) / (transition.limits.distanceAhead + EPSILON);
        double tb = (transition.limits.maxSpeedBehind - vehicle.v) / (transition.limits.distanceBehind + EPSILON);
        if (tf > 0.5 || tb > 0.5) { // less than 2 second to collision
            return 1;
        }
        else if (transition.target != NavigationState::KL && (transition.limits.distanceAhead < Config::minLaneChangeFrontDistance 
                    || -transition.limits.distanceBehind < Config::minLaneChangeBackDistance 
                    || transition.previousLimits.distanceAhead < Config::minSafeDistance
                    || -transition.previousLimits.distanceBehind < Config::minLaneChangeBackDistance)) {
            return 1; // unsafe
        }
        return  1.5 * (sigmoid(tf) - 0.5)+ 
                0.5 * (sigmoid(tb) - 0.5);
    }
};

class SpeedCost: public Cost {
public:
    SpeedCost() {};
    double operator()(Vehicle &vehicle, StateTransition &transition) const {
        double diffFront = vehicle.v - transition.limits.minSpeedAhead;
        double diffBack =  vehicle.v - transition.limits.maxSpeedBehind;
        double dist = transition.limits.distanceBehind / 5; // 5 seconds to reach as the reference
        if (fabs(dist) < EPSILON) dist = dist < 0? -EPSILON: EPSILON;
        return 1.5 * (sigmoid(diffFront) - 0.5) + 0.5 * (sigmoid(diffBack/dist) - 0.5);
    }
};

class OffTargetLaneCost: public Cost {
public:
    OffTargetLaneCost() {};
    double operator()(Vehicle &vehicle, StateTransition &transition) const {
        return 2 *(sigmoid(abs(transition.immediateLane - transition.targetLane)) - 0.5);
    }
};

class ChangeLaneCost: public Cost {
public:
    ChangeLaneCost() {};
    double operator()(Vehicle &vehicle, StateTransition &transition) const {
        return 2 *(sigmoid(abs(transition.immediateLane - transition.sourceLane)) - 0.5);
    }
};

class LaneTrafficCost: public Cost {
public:
    LaneTrafficCost() {};
    double operator()(Vehicle &vehicle, StateTransition &transition) const {
        return Config::laneFrontTrafficCostWeight * (sigmoid(transition.nVehicleAhead) - 0.5) +
                Config::laneRearTrafficCostWeight * (sigmoid(transition.nVehicleBehind) - 0.5);
    }
};

CostEvaluator::CostEvaluator() {
    addCost(new SafetyCost(), Config::safetyCostWeight);
    addCost(new SpeedCost(), Config::speedCostWeight);
    addCost(new OffTargetLaneCost(), Config::offTargetLaneCostWeight);
    addCost(new ChangeLaneCost(), Config::changetLaneCostWeight);
    addCost(new LaneTrafficCost(), Config::laneTrafficCostWeight);
}

double CostEvaluator::operator()(Vehicle &vehicle, StateTransition &transition) const {
    double cost = 0;
    double totalWeight = 0;
    if (evaluators.size() > 0) {
        for (unsigned i = 0; i < evaluators.size(); i++){
            cost += weights[i] * (*evaluators[i])(vehicle, transition);
            totalWeight += weights[i];
        }
        return cost / totalWeight;
    }
    return -1;
}
