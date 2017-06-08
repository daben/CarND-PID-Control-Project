//
//  controller.h
//  PID
//
//  Created by David Moriano on 28/05/2017.
//
//

#pragma once

#include "PID.h"
#include "twiddle.h"
#include <thread>
#include <ctime>

namespace detail
{
  // Smoother. Be careful because it will introduce a lag in the response
  // of the controller.
  struct ExponentialFilter
  {
    double alpha;
    double s;
    
    ExponentialFilter()
    : alpha(1.0), s(0.0) {}
    
    inline void Init(double alpha, double x0)
    {
      this->alpha = alpha;
      this->s = x0;
    }
    
    inline double operator()(const double x) {
      s = x * alpha + (1.0 - alpha) * s;
      return s;
    }
  };
}



struct PIDController
{
  PID     pid_steer;
  double  throttle_a;
  double  throttle_b;
  double  cte_threshold;
  double  target_speed;
  double  average_speed;
  Twiddle twiddle;
  
  // ctor
  PIDController();
  // dtor
  ~PIDController();
  // Initialize the controller with or w/o interactivity
  void Init(bool interactive=false);
  // Enable/disable paramenter learning (Twiddle)
  void SetOptimize(bool on);
  // Return optimization status
  bool IsOptimizing() const;
  // Reset controller state
  void Reset();
  // Update controller loop
  bool Update(double cte, double speed, double angle, double &steer_value, double &throttle);
    
private:
  void UpdateOptimizerParams();
  
  // Implementation details
  detail::ExponentialFilter smooth_steer_;
  detail::ExponentialFilter smooth_throttle_;
  
  std::thread io_thread_;
  bool optimizing_;
  bool must_reset_;
  size_t iterations_;
};
