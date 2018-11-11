#include <fstream>
#include <string>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "uWS/uWS.h"
#include "utils/json.hpp"
#include "Eigen/Core"
#include "Eigen/QR"
#include "utils/utils.h"
#include "model/Road.hpp"
#include "model/Vehicle.hpp"
#include "control/Navigator.hpp"

static string configFile = "";

// Don't use std namespace as it might cause g++ 7 to to use std::array for array in json.hpp, and fail
// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in std::string format will be returned,
// else the empty std::string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main(int argc, char *argv[]) {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal std::vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  
  // Process command line options
  for (int i = 1; i < argc; i++) {
    if (std::string((argv[i])) == "-config") { // Set the number of particles
      configFile = argv[++i];
    } else if (std::string((argv[i])) == "-fast") { // set std GPS deviation
      configFile = "../config-fast.json";
    } else if (std::string((argv[i])) == "-safe") { // set std GPS deviation
      configFile = "../config-safe.json";
    } else {
      std::cerr << "Unknown option: " << argv[i] << std::endl;
      exit(-1);
    }
  }

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (std::getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  Road road(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, max_s);
  Road::setCurrentRoad(road);
  Navigator navigator;

  h.onMessage(
      [&road, &navigator](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = std::string(data).substr(0, length);
        //std::cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
          auto s = hasData(data);

          if (s != "")
          {
            auto j = json::parse(s);

            std::string event = j[0].get<std::string>();

            if (event == "telemetry")
            {
              // j[1] is the data JSON object

              // Main car's localization Data
              double car_x = j[1]["x"];
              double car_y = j[1]["y"];
              double car_s = j[1]["s"];
              double car_d = j[1]["d"];
              double car_yaw = j[1]["yaw"];
              double car_speed = j[1]["speed"];

              // Previous path data given to the Planner
              vector<double> previous_path_x = j[1]["previous_path_x"];
              vector<double> previous_path_y = j[1]["previous_path_y"];
              // Previous path's end s and d values
              double end_path_s = j[1]["end_path_s"];
              double end_path_d = j[1]["end_path_d"];

              // Sensor Fusion Data, a list of all other cars on the same side of the road.
              std::vector<std::vector<double>> sensor_fusion = j[1]["sensor_fusion"];
              std::vector<Vehicle> fusion;

              for (auto it: sensor_fusion) {
                Vehicle tmp(it);
                fusion.push_back(tmp);
              }

              if (!navigator.isVehicleInitialized()) {
                std::cout << "Initialize vehicle" << std::endl;
                navigator.initializeVehicle(car_x, car_y, car_s, car_d, car_yaw, car_speed);
              }
              
              navigator.update(fusion, previous_path_x, previous_path_y);

#ifdef DEBUG_OUT
              std::cout << "Current: x: " << car_x << " y: " << car_y << " s: " << car_s << " d: " << car_d <<  " v: " << MpH2MpS(car_speed) << " yaw: "  << car_yaw << std::endl;
              
              cout <<  "Vehicle prediction start: x: " << navigator.getVehicle()->x << " y: " << navigator.getVehicle()->y << " s: " << navigator.getVehicle()->s << " d: " << navigator.getVehicle()->d
                   << " v: " << MpS2MpH(navigator.getVehicle()->v) << " vs: " << MpS2MpH(navigator.getVehicle()->vs) << " vd: " << MpS2MpH(navigator.getVehicle()->vd) << " a: " 
                   << MpS2MpH(navigator.getVehicle()->a)  << std::endl;
#endif
              std::vector<std::vector<double>> next_trjectory = navigator.navigate();
              
              json msgJson;

              // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
              msgJson["next_x"] = next_trjectory[0];
              msgJson["next_y"] = next_trjectory[1];

              auto msg = "42[\"control\"," + msgJson.dump() + "]";

              //this_thread::sleep_for(chrono::milliseconds(1000));
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          }
          else
          {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    }
    else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!, loading config from: " << configFile << endl;
    if (!configFile.empty()) {
      Config::load(configFile);
    }
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  }
  else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
