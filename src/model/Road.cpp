#include "Road.hpp"
#include "../utils/utils.h"

using namespace std;

Road *Road::currentRoad;

Road::Road(const std::vector<double> &s, const std::vector<double> &x, const std::vector<double> &y,
           const std::vector<double> &nx, const std::vector<double> &ny, const double maxs): 
                            road_s(s), road_x(x), road_y(y), road_nx(nx), road_ny(ny), maxS(maxs) {
    splineSX.set_points(s, x);
    splineSY.set_points(s, y);
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
    return s;
}

vector<double> Road::getXY(const double s, const double d) {
    // Here we use the spline between S and X, and spline between S and Y to compute the normal, x and y
    // x = splineSX(S) + normal[0] * d
    //  = splineSY(S) + normal[1] * d
    int prev_wp = -1;
    double S = normalizeS(s);
    vector<double> norm = getNormalAtS(S);
    return {splineSX(S) + d * norm[0], splineSY(S) + d * norm[1]};
}

vector<double> Road::getNormalAtS(double s) {
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
// vector<double> Road::getXY2(const double s, const double d) {
//     double S = normalizeS(s);
//     int prev_wp = -1;

//     while (S > road_s[prev_wp + 1] && (prev_wp < (int) (road_s.size() - 1))) {
//         prev_wp++;
//     }

//     int wp2 = (prev_wp + 1) % road_x.size();

//     double sx = road_x[prev_wp]
//         + (S - road_s[prev_wp]) * (road_x[wp2] - road_x[prev_wp]) / (road_s[wp2] - road_s[prev_wp]);
//     double sy = road_y[prev_wp]
//         + (S - road_s[prev_wp]) * (road_y[wp2] - road_y[prev_wp]) / (road_s[wp2] - road_s[prev_wp]);

//     double dvx = (road_y[wp2] - road_y[prev_wp]);
//     double dvy = (road_x[wp2] - road_x[prev_wp]);

//     double dvl = length(dvx, dvy);

//     dvx /= dvl;
//     dvy /= dvl;

//     if (d > 0) {
//         dvy = -dvy;
//     }
//     else {
//         dvx = -dvx;
//     }

//     double tmp = fabs(d);

//     double x = sx + tmp * dvx;
//     double y = sy + tmp * dvy;

//     return {x, y};
// }
//vector<double> Road::getDirectionAtS(double s) {
    // int i = 0;
    // for (; i < (int) road_s.size(); i++) {
    //     if (s >= road_s[i])
    //         break;
    // }

    // if (i == (int) road_s.size() - 1) {
    //     return {-road_ny[i], road_nx[i]};
    // }

    // double f = (s - road_s[i]) / (road_s[i+1] - road_s[i]);

    // // Linear interpolation
    // return unitv(-(road_ny[i] * f + road_ny[i+1] * (1 - f)), road_nx[i] * f + road_nx[i+1] * (1 - f));
//}

vector<double> Road::getFrenetDirection(double x, double y, double s, double d, double dx, double dy) {
    int next_wp = NextWaypoint(x, y, dx, dy);
    vector<double> F = this->getFrenet(x + dx, y + dy, dx, dy);
    return unitv(F[0] - s, F[1] - d);
}