#include <uWS/uWS.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <atomic>

#include "json.hpp"
#include "controller.h"


// for convenience
using json = nlohmann::json;

namespace /* anonymous */
{
  // Checks if the SocketIO event has JSON data.
  // If there is data the JSON object in string format will be returned,
  // else the empty string "" will be returned.
  std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
      return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
  }

  // Global variable to fix a race condition when resetting the simulator.
  static std::atomic<bool> resetting_(false);

  // Helper to reset the simulator
  void resetDriving(uWS::WebSocket<uWS::SERVER> &ws) {
    std::string msg("42[\"reset\", {}]");
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    std::cout << "** Resetting simulator **" << std::endl;
    resetting_ = true;
  }
} // (anonymous)


int main(int argc, char * argv[])
{
  uWS::Hub h;

  bool optimize{false};
  bool interactive{false};

  for(int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-l") || !strcmp(argv[i], "-learn")) {
      optimize = true;
    } else if (!strcmp(argv[i], "-i")) {
      interactive = true;
    } else {
      std::cerr << "Usage: " << argv[0] << " [Options]" << std::endl
      << "Options: " << std::endl
      << "  -l,-learn         Optimize params." << std::endl
      << "  -i                Interactive." << std::endl
      << std::endl;
      return 1;
    }
  }

  // The controller logic has been move to the class PIDController.
  // See "controller.h" and "controller.cpp".
  PIDController controller;
  controller.Init(interactive);
  controller.SetOptimize(optimize);

  h.onMessage([&controller]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    if (resetting_) {
      // Ignore all the commands in the buffer until the reset is completed
      return;
    }

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          // default values
          double steer_value = 0.0;
          double throttle_value = 0.3;

          if (!controller.Update(cte, speed, angle, steer_value, throttle_value))
          {
            // The controller can't update, the errors may be too big.
            // Reset the simulator and try again.
            resetDriving(ws);
            return;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h, &controller](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    // Reset the controller
    controller.Reset();
    if (resetting_) resetting_ = false;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
