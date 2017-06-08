#pragma once


class PID {
public:
  /// Proportional error
  double p_error;
  /// Integral error
  double i_error;
  /// Derivative error
  double d_error;

  /// Proportional gain
  double Kp;
  /// Integral gain
  double Ki;
  /// Derivative gain
  double Kd;
  
  /// Exponential smoothing for the integral error.
  /// This parameters dimisnishes the effect of past errors.
  /// Default value is 1.0 which makes this a standard PID.
  double i_alpha;
  
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double i_alpha=1.0);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double error, double dt=1.0);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};
