#include "Cost.hpp"

using namespace std;

class SafetyCost: public Cost {
public:
    SafetyCost() {};
    double operator()(const Vehicle &vehicle, const int lane, Limits &limits) const {
        double tf = (vehicle.v - limits.minSpeedAhead) / (limits.distanceAhead + EPSILON);
        double tb = (limits.maxSpeedBehind - vehicle.v) / (limits.distanceBehind + EPSILON);
        if (tf > 0.5 || tb > 0.5) { // less than 2 second to collision
            return 1;
        }
        else if (lane != vehicle.laneLeft && lane != vehicle.laneRight && (limits.distanceAhead < Config::minLaneChangeFrontDistance 
                    || -limits.distanceBehind < Config::minLaneChangeBackDistance)) {
            return 1; // unsafe
        }
        return 1.5 * (sigmoid(tf) - 0.5) + 0.5 * (sigmoid(tb) - 0.5);
    }
};

class SpeedCost: public Cost {
public:
    SpeedCost() {};
    double operator()(const Vehicle &vehicle, const int lane, Limits &limits) const {
        double diffFront = vehicle.v - limits.minSpeedAhead;
        double diffBack =  vehicle.v - limits.maxSpeedBehind;
        double dist = limits.distanceBehind / 5; // 5 seconds to reach as the reference
        if (fabs(dist) < EPSILON) dist = dist < 0? -EPSILON: EPSILON;
        return 1.5 * (sigmoid(diffFront) - 0.5) + 0.5 * (sigmoid(diffBack/dist) - 0.5);
    }
};

class ChangeLaneCost: public Cost {
public:
    ChangeLaneCost() {};
    double operator()(const Vehicle &vehicle, const int lane, Limits &limits) const {
        return 2 *(sigmoid(abs(vehicle.laneRight + vehicle.laneLeft - 2 * lane) / 2) - 0.5);
    }
};

class LaneTrafficCost: public Cost {
public:
    LaneTrafficCost() {};
    double operator()(const Vehicle &vehicle, const int lane, Limits &limits) const {
        return Config::laneFrontTrafficCostWeight * (sigmoid(limits.nVehicleAhead) - 0.5) +
                Config::laneRearTrafficCostWeight * (sigmoid(limits.nVehicleBehind) - 0.5);
    }
};

CostEvaluator::CostEvaluator() {
    addCost(new SafetyCost(), Config::safetyCostWeight);
    addCost(new SpeedCost(), Config::speedCostWeight);
    addCost(new ChangeLaneCost(), Config::changetLaneCostWeight);
    addCost(new LaneTrafficCost(), Config::laneTrafficCostWeight);
}

double CostEvaluator::operator()(const Vehicle &vehicle, const int lane, Limits &limits) const {
    double cost = 0;
    double totalWeight = 0;
    if (evaluators.size() > 0) {
        for (unsigned i = 0; i < evaluators.size(); i++){
            cost += weights[i] * (*evaluators[i])(vehicle, lane, limits);
            totalWeight += weights[i];
        }
        return limits.cost = cost / totalWeight;
    }
    return limits.cost = -1;
}
