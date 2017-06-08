//
//  controller.cpp
//  PID
//
//  Created by David Moriano on 28/05/2017.
//
//

#include "controller.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>


namespace
{
  // For converting back and forth between radians and degrees.
  constexpr double pi() { return M_PI; }
  constexpr double deg2rad(const double x) { return x * pi() / 180.0; }
  constexpr double rad2deg(const double x) { return x * 180.0 / pi(); }

  // clamp a value inside a range. This function is actually available
  // in the stdlib since C++17.
  template<class T>
  constexpr const T& clamp(const T& v, const T& lo, const T& hi) {
    return std::max(lo, std::min(v, hi));
  }

  // Debugging
  inline std::ostream & operator<<(std::ostream &s, const PID &pid) {
    std::ios::fmtflags flags(s.flags());
    s << std::scientific << std::setprecision(3)
      << "PID[" << pid.Kp << ", " << pid.Ki << ", " << pid.Kd << "; "
      << "A=" << pid.i_alpha << "; "
      << "E=" << pid.Kp*pid.p_error << ", "
              << pid.Ki*pid.i_error << ", "
              << pid.Kd*pid.d_error << "]";
    s.flags(flags);
    return s;
  }
} // (anonymous)


PIDController::PIDController() {
}

PIDController::~PIDController() {
}

void PIDController::Init(bool interactive) {
  
  // Here I'm cautius with the target speed because of the latency problems
  // with the simulator.
  target_speed = 50;
  // Initialize the PID controller with values tuned for this target speed.
  // For higher speeds a lower P is needed.
  pid_steer.Init(0.106, 0.001, 2.4, 0.95);
  
  if (target_speed > 65)
    pid_steer.Init(0.85, 0.0004, 2.2, 0.95);
  
  // Throttle control
  throttle_a = 0.2;
  throttle_b = 0.8;
  
  // Initial average speed
  average_speed = 0;
  
  // Smoothing
  smooth_steer_.Init(0.99, 0.0);
  smooth_throttle_.Init(0.99, 1.0);
  
  // CTE threshold to signal that we are out of the road
  cte_threshold = 5;
  // Optimization disabled by default
  optimizing_ = false;
  // Reset flag
  must_reset_ = false;
  // Iterations counter
  iterations_ = 0;

  // Interactive thread to read stdin commands
  if (interactive &&
  // not initialized already
  io_thread_.get_id() == std::thread::id()) {
    // Create the user input thread
    io_thread_ = std::thread([&]{
      std::string line;
      while (std::cin) {
        std::cerr << "Steer: " << pid_steer
                  << " Throttle: a: " << throttle_a << ", b: " << throttle_b
                  << " Target " << target_speed;
        if (optimizing_) {
          std::cerr << " Twiddle "
                    << (twiddle.current_error / twiddle.count)
                    << " / " << twiddle.best_error;
        }
        std::cerr << std::endl;

        std::cerr << "> ";
        std::flush(std::cerr);
        std::getline(std::cin, line);
        std::stringstream s(line);

        std::string command;
        double value;
        s >> command;
        // make lowercase
        std::transform(command.begin(), command.end(), command.begin(), ::tolower);

        if (command == "r" or command == "reset") {
          must_reset_ = true;
          continue;
        }
        else if (command == "q" or command == "quit") {
          exit(0);
        }
        else if (command == "h" or command == "help") {

        }

        s >> value;
        if (!s.fail())
        {
          if (command == "p") {
            value *= 1e-2;
            pid_steer.Kp = value;
            if (optimizing_)
              twiddle.params[0] = value;
          }
          else if (command == "i") {
            value *= 1e-4;
            pid_steer.Ki = value;
            if (optimizing_)
              twiddle.params[1] = value;
          }
          else if (command == "d") {
            pid_steer.Kd = value;
            if (optimizing_)
              twiddle.params[2] = value;
          }
          else if (command == "ia") {
            pid_steer.i_alpha = value;
            //if (optimizing_)
            //  twiddle.params[3] = value;
          }
          else if (command == "a") {
            throttle_a = value;
            //if (optimizing_)
            //  twiddle.params[3] = value;
          }
          else if (command == "b") {
            throttle_b = value;
            //if (optimizing_)
            //  twiddle.params[4] = value;
          }
          else if (command == "s") {
            target_speed = value;
            //if (optimizing_)
            //  twiddle.params[5] = value;
          }
          else if (command == "tw") {
            SetOptimize(bool(value));
          }
          else if (command == "l") {
            twiddle.cycle_size = int(value);
          }
          else if (command == "steps") {
            twiddle.steps[0] = value;
            for(int i=1; i<twiddle.steps.size(); ++i) {
              s >> twiddle.steps[i];
            }
          }
          else {
            /* ignore */
            continue;
          }

          Reset();
        }
      }

    });
  }
}

bool PIDController::IsOptimizing() const {
  return optimizing_;
}

void PIDController::SetOptimize(bool on) {
  if (on != optimizing_) {
    optimizing_ = on;
    // Initialize twiddle
    if (on) twiddle.Init(// initial params
                         { pid_steer.Kp, pid_steer.Ki, pid_steer.Kd,
                           //pid_steer.i_alpha, throttle_a, throttle_b
                         },
                         // steps
                         { 0.01, 1e-4, 0.1,
                           //0.01, 0.1, 0.1
                         },
                         250);
  }
}

void PIDController::UpdateOptimizerParams()
{
  // Update the params
  pid_steer.Init(/* Kp */twiddle.params[0],
                 /* Ki */twiddle.params[1],
                 /* Kd */twiddle.params[2],
                 /* i_alpha */ 0.95);
  //throttle_a = twiddle.params[4];
  //throttle_b = twiddle.params[5];
}

void PIDController::Reset()
{
  // Reset PID errors
  pid_steer.p_error = 0;
  pid_steer.i_error = 0;
  pid_steer.d_error = 0;
  iterations_ = 0;
  average_speed = 0;
}

bool PIDController::Update(double cte, double speed, double angle, double &steer_value, double &throttle)
{
  /*
   * TODO: Calculate steering value here, remember the steering value is
   * [-1, 1].
   *
   * NOTE: Feel free to play around with the throttle and speed. Maybe use
   * another PID controller to control the speed!
   */

  // Reset the simulator and the controller if we get out of the road
  // while optimizing the parameters or if the user requested it
  if (must_reset_ || (optimizing_ && fabs(cte) > cte_threshold)) {
    // set the flag off
    must_reset_ = false;
    
    // Reset the controller
    Reset();
    
    if (optimizing_) {
      // Reset twiddle
      twiddle.ResetCycle();
      // Sync the parameters
      UpdateOptimizerParams();
    }

    // Signal the caller that we can't update.
    return false;
  }
  
  // Time delta, we assume 1.
  constexpr double dt = 1.0;
  
  // Update average speed
  iterations_ ++;
  average_speed = (average_speed * (iterations_ - 1) + speed) / iterations_;

  if (optimizing_)
  {
    // Minimize the cte and the average speed, empirically weighted to favour the cte
    std::cout << "Twiddle Error "
              << fabs(cte) << " + "
              << fabs(target_speed - average_speed) / target_speed
              << std::endl;
    if (twiddle.Update(fabs(cte) + fabs(target_speed - average_speed) / target_speed)) {
      // Update our parameters
      UpdateOptimizerParams();
      // Reset the errors
      Reset();
    }
    
    // Stop condition
    if (twiddle.TotalStep() < 1e-4) {
      optimizing_ = false;
    }
  }

  // Update the PID controller with the current cross track error.
  pid_steer.UpdateError(cte, dt);
  // Set the steering angle to the opposite of the error.
  steer_value = -pid_steer.TotalError();
  // Here I use the tanh to smoothly clamp into [-1, 1].
  steer_value = tanh(steer_value);
  // Temporal smoothing
  steer_value = smooth_steer_(steer_value);

  // Here I handle the throttle with two direct P-controllers one accelerating
  // and the other braking. The acceleration is proportional to the speed
  // error, the braking is proportional to cte and exponential to the speed.
  //
  // Changing the gains of these two controllers you can tune the driving style.
  const double speed_error = (target_speed - speed + /*margin*/5);
  throttle = throttle_a * speed_error
           - throttle_b * fabs(cte) * (exp(fabs(speed/100) * 1.1) - 1);
  // Clamp the throttle to [-1, 1].
  throttle = clamp(throttle, -1.0, 1.0);
  // Temporal smoothing
  throttle = smooth_throttle_(throttle);

  // Debug
  std::cout
    << std::fixed << std::setprecision(2)
    << "CTE: " << std::setw(5) << cte / speed
    << " dt: " << std::setw(5) << dt
    << " Throttle Error: " << std::setw(6) << speed_error
    << " Speed: " << std::setw(6) << speed
    << " Average Speed: " << std::setw(6) << average_speed
    << " Angle: " << std::setw(6) << angle
    << " Steering: " << std::setw(6) << steer_value
    << " Throttle: " << std::setw(6) << throttle
    << std::endl
    << " pid_steer: " << std::setw(6) << pid_steer
    << " throttle: " << std::setw(6) << throttle_a << ", " << std::setw(6) << throttle_b
    << " target: " << std::setw(6) << target_speed
    << std::endl;

  return true;
}
