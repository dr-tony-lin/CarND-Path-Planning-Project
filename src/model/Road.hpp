#ifndef _MAP_HPP_
#define _MAP_HPP_

#include <vector>
#include "../utils/Config.hpp"
#include "../utils/utils.h"
#include "../utils/spline.h"

/**
 * A navigation road map.
 */ 
class Road {
private:
    static Road *currentRoad;
    std::vector<double> road_s;
    std::vector<double> road_x;
    std::vector<double> road_y;
    std::vector<double> road_nx;
    std::vector<double> road_ny;
    // Since s to waypoints'x, y mapping is one to one, we can use the spline library (from Tino Kluge)
    // to compute x, y from s, d
    tk::spline splineSX; // Spline for s, x 
    tk::spline splineSY; // spline for s, y
    double laneWidth = Config::laneWidth;
    double speedLimit = Config::maxSpeed;
    double maxS;
    int lanes = Config::numberOfLanes;

protected:
    /**
     * Compute frenet coordinate
     * @param x the x coordinate
     * @param y the y coordinate
     * @param wp the way point closest to the point
     */ 
    std::vector<double> getFrenet(const double x, const double y, const int next_wp);

public:
    /**
     * Constructor
     * @param s the s coordinate vector
     * @param x the x coordinates
     * @param y the y coordinates
     * @param nx the x component of the normal (the d vector) of the road pointing to the left along the positive s direction
     * @param nx the x component of the normal of the road pointing pointing to the left along the positive s direction
     */ 
    Road(const std::vector<double> &s, const std::vector<double> &x, const std::vector<double> &y,
         const std::vector<double> &nx, const std::vector<double> &ny, const double maxS, const double lanes=Config::numberOfLanes);

    /**
     * NOrmalize the S coordinate as the road is circular
     */ 
    double normalizeS(const double s);

    /**
     * Set the current road the car is driving
     * @param road the road
     */ 
    static void setCurrentRoad(Road &road) {
        currentRoad = &road;
    }
    
    /**
     * Get the current road the car is driving
     */ 
    static Road& current() {
        return *currentRoad;
    }

    /**
     * Return the number of lanes the road has at s
     * @param s the location
     */ 
    int getNumberOfLanesAt(const double s) { return lanes;};

    /**
     * Return the maximum number of lanes
     */ 
    int numberOfLanes() { return lanes;};

    /**
     * Return the speed limit
     */ 
    double getSpeedLimit() { return speedLimit;};

    /**
     * Return the lane's width
     */ 
    double getLaneWidth() { return laneWidth;};

    /**
     * Return the lane given d coordinate
     * @param d the d coordinate
     */
    int dToLane(const double d) {
        return int(d / laneWidth);
    }

    /**
     * Return the lane center's d coordinate
     * @param lane the lane
     */ 
    double laneToCenterD(const int lane) {
        return lane * laneWidth + 0.5 * laneWidth;
    }

    /**
     * Set the speed limit
     * @param limit
     */ 
    void setSpeedLimit(double limit) { speedLimit = limit;};

    /**
     * Find the waypoint closest to the given location
     * @param x the x coordinate
     * @param y the y coordinate
     */ 
    unsigned int ClosestWaypoint(const double x, const double y);

    /**
     * Find the next waypoint to the given vehicle location
     * @param x the x coordinate
     * @param y the y coordinate
     * @param theta the vehicle' heading, (0 to 2PI)
     */ 
    int NextWaypoint(const double x, const double y, const double theta);

    /**
     * Find the next waypoint to the given location
     * @param x the x coordinate
     * @param y the y coordinate
     * @param dx the x element of the vehicle' heading, must be a unit vector 
     * @param dy the y element of the vehicle' heading, must be a unit vector
     */ 
    int NextWaypoint(const double x, const double y, const double dx, const double dy);

    /**
     * Compute frenet coordinate
     * @param x the x coordinate
     * @param y the y coordinate
     * @param theta the vehicle' heading, (0 to 2PI)
     */ 
    std::vector<double> getFrenet(const double x, const double y, const double theta);

    /**
     * Compute frenet coordinate
     * @param x the x coordinate
     * @param y the y coordinate
     * @param dx the x element of the vehicle' heading, must be a unit vector 
     * @param dy the y element of the vehicle' heading, must be a unit vector
     */ 
    std::vector<double> getFrenet(const double x, const double y, const double dx, const double dy);

    /**
     * Compute the x, and y coordinate
     * @param s the frenet coordinate s
     * @param d the frenet coordinate d
     */ 
    std::vector<double> getXY(const double s, const double d);

    /**
     * Get normal of the road's trajectory at frenet location s, 
     * the normal will point to the right when traversing along positive s direstion
     * @param s the frenet S coordinate
     */ 
    std::vector<double> getNormalAtS(double s);

    /**
     * Get direction the road at frenet location s
     * @param s the frenet S coordinate
     */ 
    std::vector<double> getDirectionAtS(double s);

    /**
     * Compute frenet directional vector (ds, dd)
     * @param x the x coordinate
     * @param y the y coordinate
     * @param s the corresponding frenet coordinate s
     * @param d the corresponding frenet coordinate d
     * @param dx the x element of the vehicle' heading, must be a unit vector 
     * @param dy the y element of the vehicle' heading, must be a unit vector
     */ 
    std::vector<double>getFrenetDirection(double x, double y, double s, double d, double dx, double dy);

    /**
     * Compute distance between s1, and s2, account for circular road
     */
    double distanceS(const double s1, const double s2);
};

#endif