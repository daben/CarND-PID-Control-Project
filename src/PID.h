#ifndef PID_H
#define PID_H

#include <chrono>

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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
private:
  using clock = std::chrono::steady_clock;
  clock::time_point time_point_;
};

#endif /* PID_H */
