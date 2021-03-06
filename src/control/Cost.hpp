#ifndef _CONTROL_COST_H
#define _CONTROL_COST_H
#include <vector>
#include "../model/Vehicle.hpp"
#include "../model/State.hpp"
#include "../model/Vehicle.hpp"

using namespace std;

class Cost {
public:
    Cost() {};
    virtual double operator()(const Vehicle &vehicle, const int lane, Limits &limits) const = 0;
};

class CostEvaluator: public Cost {
    vector<Cost*> evaluators;
    vector<double> weights;
public:
    CostEvaluator();
    double operator()(const Vehicle &vehicle, const int lane, Limits &limits) const ;
    void addCost(Cost* cost, double weight) {
        evaluators.push_back(cost);
        weights.push_back(weight);
    }
};

#endif