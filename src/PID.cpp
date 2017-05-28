#include "PID.h"
#include <stdio.h>

/*
* TODO: Complete the PID class.
*/

PID::PID()
{
  Init(0, 0, 0);
}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  // Set gains
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  
  // Initialize the errors
  p_error = 0;
  i_error = 0;
  d_error = 0;
  
  time_point_ = clock::now();
}

void PID::UpdateError(double cte) {

  // We could assume that the loop interval time is dt=0.1 secs because
  // the simulator emits telemetry at 10Hz.

  const auto now = clock::now();
  double dt = std::chrono::duration_cast<std::chrono::milliseconds>
                (now - time_point_).count() * 1e-3;
  time_point_ = now;
  
  // differential error. Note that p_error holds the previous cte
  d_error = (cte - p_error) / dt;
  // accumulated error since the beginning
  i_error += cte * dt;
  // proportional error, just the current cte
  p_error = cte;
}

double PID::TotalError() {
  // The total error is the sum of the P, I, D errors weighted Kp, Ki, Kd.
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

