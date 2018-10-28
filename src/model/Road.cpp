#include "Road.hpp"
#include "../utils/utils.h"

using namespace std;

Road *Road::currentRoad;

Road::Road(const std::vector<double> &s, const std::vector<double> &x, const std::vector<double> &y,
           const std::vector<double> &nx, const std::vector<double> &ny, const double maxs): 
                            road_s(s), road_x(x), road_y(y), road_nx(nx), road_ny(ny), maxS(maxs) {
    vector<double> S;
    vector<double> X;
    vector<double> Y;
    // Add the end point to the start for continuity
    S.push_back(s[s.size() - 2] - s.back());
    X.push_back(x.back());
    Y.push_back(y.back());
    // Add the waypoints
    S.insert(S.end(), s.begin(), s.end());
    X.insert(X.end(), x.begin(), x.end());
    Y.insert(Y.end(), y.begin(), y.end());
    // append the starting point for continuity
    S.push_back(maxS);
    X.push_back(x[0]);
    Y.push_back(y[0]);
    // S.push_back(s[1] + maxS);
    // X.push_back(x[1]);
    // Y.push_back(y[2]);
    splineSX.set_points(S, X);
    splineSY.set_points(S, Y);
}

unsigned int Road::ClosestWaypoint(const double x, const double y) {
    double closestLen = 100000; //large number
    unsigned int closestWaypoint = 0;

    for (unsigned int i = 0; i < road_x.size(); i++) {
        double map_x = road_x[i];
        double map_y = road_y[i];
        double dist = distance(x, y, map_x, map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

int Road::NextWaypoint(double x, double y, double theta) {
    unsigned int closestWaypoint = ClosestWaypoint(x, y);

    double map_x = road_x[closestWaypoint];
    double map_y = road_y[closestWaypoint];
    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta - heading);
    angle = fmin(2 * M_PI - angle, angle);

    if (angle > M_PI / 4) {
        closestWaypoint++;
        if (closestWaypoint == road_x.size()) {
            closestWaypoint = 0;
        }
    }

  return closestWaypoint;
}

int Road::NextWaypoint(const double x, const double y, const double dx, const double dy) {
    unsigned int closestWaypoint = ClosestWaypoint(x, y);

    double map_x = road_x[closestWaypoint];
    double map_y = road_y[closestWaypoint];

    vector<double> h = unitv(map_x - x, map_y - y);

    if (fabs(dot(dx, dy, h[0], h[1])) < fabs(cross(dx, dy, h[0], h[1]))) {
        closestWaypoint++;
        if (closestWaypoint == road_x.size()) {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

vector<double> Road::getFrenet(const double x, const double y, const int next_wp) {
    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0) {
        prev_wp = road_x.size() - 1;
    }

    vector<double> v = unitv(road_x[next_wp] - road_x[prev_wp], road_y[next_wp] - road_y[prev_wp]);
    double x_x = x - road_x[prev_wp];
    double x_y = y - road_y[prev_wp];

    double frenet_d = cross(x_x, x_y, v[0], v[1]);
    double frenet_s = road_s[prev_wp] + dot(x_x, x_y, v[0], v[1]);

    return {frenet_s, frenet_d};
}

vector<double> Road::getFrenet(const double x, const double y, const double theta) {
    int next_wp = NextWaypoint(x, y, theta);
    return getFrenet(x, y, next_wp);
}

std::vector<double> Road::getFrenet(const double x, const double y, const double dx, const double dy) {
    int next_wp = NextWaypoint(x, y, dx, dy);
    return getFrenet(x, y, next_wp);
}

double Road::normalizeS(const double s) {
    if (s > maxS) {
        return s - maxS;
    }
    else if (s < 0) {
        return s + maxS;
    }
    return s;
}
double Road::distanceS(const double s1, const double s2) {
    double distance = s1 - s2;
    if (fabs(distance) > 0.5 * maxS) {
        distance = distance > 0? distance - maxS: distance + maxS;
    }

    return distance;
}

vector<double> Road::getXY(const double s, const double d) {
    // Here we use the spline between S and X, and spline between S and Y to compute the normal, x and y
    // x = splineSX(S) + normal[0] * d
    // y = splineSY(S) + normal[1] * d
    // It is more efficient than the original algorithm (593 VS 696 millies for 10 million operarions)
    int prev_wp = -1;
    double S = normalizeS(s);
    vector<double> norm = getNormalAtS(S);
    return {splineSX(S) + d * norm[0], splineSY(S) + d * norm[1]};
}

vector<double> Road::getNormalAtS(double s) {
    // Compute the normal using the first derivative of splineSX, and SplineSY
    // It is bit slower than the original algrithm (344 vs 206 millies for 10 million operations)
    // But seems be more accurate
    double S = normalizeS(s);
    double dx = splineSX.deriv(1, S);
    double dy = splineSY.deriv(1, S);
    return unitv(dy, -dx);
}

vector<double> Road::getDirectionAtS(double s) {
    double S = normalizeS(s);
    double dx = splineSX.deriv(1, S);
    double dy = splineSY.deriv(1, S);
    return unitv(dx, dy);
}

vector<double> Road::getFrenetDirection(double x, double y, double s, double d, double dx, double dy) {
    vector<double> F = this->getFrenet(x + dx, y + dy, dx, dy);
    return unitv(F[0] - s, F[1] - d);
}
