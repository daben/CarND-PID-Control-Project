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

void PID::Init(double Kp_, double Ki_, double Kd_, double i_alpha_) {
  // Set gains
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  i_alpha = i_alpha_;
  
  // Initialize the errors
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double error, double dt) {
  // differential error. Note that p_error holds the previous cte
  d_error = (error - p_error) / dt;
  // accumulated error since the beginning
  i_error = error * dt + i_alpha * i_error;
  // proportional error, just the current cte
  p_error = error;
}

double PID::TotalError() {
  // The total error is the sum of the P, I, D errors weighted Kp, Ki, Kd.
  return Kp * p_error + Ki * i_error + Kd * d_error;
}
