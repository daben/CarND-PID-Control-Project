#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include "twiddle.h"
#include <cmath>
#include <algorithm>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// clamp a value inside a range. This function is actually available
// in the stdlib since C++17.
template<class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi)
{
  return v < lo ? lo : v > hi ? hi : v;
}


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

// reset the simulator
void resetDriving(uWS::WebSocket<uWS::SERVER> &ws) {
  std::string msg("42[\"reset\", {}]");
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;
  
  
  PID pid_steer;
  PID pid_speed;
  
  Twiddle twiddle;
  bool learning_to_drive = false;
  
  // Initialize the PID controllers.
  // Init with previously learned parameters
  pid_steer.Init(3.1557e-01, 1.2500e-03, 1.1074e-01);
  
  //pid_speed.Init(0.3, 0.0001, 0.0);
  
  if (learning_to_drive) {
    twiddle.Init(/*inital params*/    {pid_steer.Kp, pid_steer.Ki, pid_steer.Kd},
                 /*perturbation*/     {0.1, 0.0001, 0.001},
                 /*cycle size @10Hz*/ 10 * 10);
  }
  
  h.onMessage([&pid_steer, &pid_speed, &twiddle, &learning_to_drive]
              (uWS::WebSocket<uWS::SERVER>
#ifdef WS_USE_POINTER
            *ws_p,
#else
            ws,
#endif
            char *data, size_t length, uWS::OpCode opCode) {
#ifdef WS_USE_POINTER
    auto & ws = *ws_p;
#endif
    
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

          double steer_value = 0.0;
          double throttle_value = 0.3;
          
          /*
          * DONE: Calculate steering value here, remember the steering value is
          * [-1, 1].
          * 
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          if (learning_to_drive) {
            
            // If cross track error is too big, reset the learning
            if (fabs(cte) > 5) {
              resetDriving(ws);
              pid_steer.p_error = 0;
              pid_steer.i_error = 0;
              pid_steer.d_error = 0;
              twiddle.Reset();
            }
            
            if (twiddle.Update(cte)) {

              printf("Twiddle E = %.3e TOL = %5.2f "
                     "  {Kp = %8.4e, Ki = %8.4e, Kd = %8.4e}"
                     "   P[%d] [%8.4f, %8.4f, %8.4f] CTE = %8.3f\n",
                     twiddle.best_error,
                     twiddle.TotalPerturbation(),
                     twiddle.params[0], twiddle.params[1], twiddle.params[2],
                     twiddle.current_param,
                     twiddle.perturbation[0], twiddle.perturbation[1], twiddle.perturbation[2],
                     pid_steer.i_error);
              
              pid_steer.Kp = twiddle.params[0];
              pid_steer.Ki = twiddle.params[1];
              pid_steer.Kd = twiddle.params[2];
              pid_steer.p_error = 0;
              pid_steer.i_error = 0;
              pid_steer.d_error = 0;

              if (twiddle.TotalPerturbation() < 0.01) {
                learning_to_drive = false;
              }
            }
          }
          
          // Update the PID controller with the current cross track error
          pid_steer.UpdateError(cte);
          
          // Set the steering angle to the opposite of the error, clamped inside its range
          steer_value = clamp(-pid_steer.TotalError(), -1.0, 1.0);
          
          // TODO: throttle
          //pid_speed.UpdateError(100 - speed);
          //throttle_value = clamp(-pid_speed.TotalError(), -1.0, 1.0);

          // DEBUG
          if (!learning_to_drive) {
            std::cout << "CTE: " << cte
                      << " Steering Value: " << steer_value
                      << " Throttle Value: " << throttle_value
                      << std::endl;
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

#ifdef WS_USE_POINTER
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws_p, uWS::HttpRequest req) {
#else
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
#endif
    std::cout << "Connected!!!" << std::endl;
  });

#ifdef WS_USE_POINTER
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws_p, int code, char *message, size_t length) {
    auto & ws = *ws_p;
#else
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
#endif
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
