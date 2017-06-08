//
//  twiddle.h
//  PID
//
//  Created by David Moriano on 26/05/2017.
//
//

#pragma once

#include <valarray>

class Twiddle
{
public: // Make this fields public to easy the use
  using array = std::valarray<double>;

  array   params;
  array   steps;
  array   best_params;
  array   best_steps;
  double  current_error;
  double  best_error;
  int     cycle_size;
  double  delta{0.05};
  int     count;

private:
  int     current_param;
  int     state;

public:
  Twiddle() {}
  ~Twiddle() {}

  void Init(const array& params, const array& steps, const int cycle);
  bool Update(double error);
  void ResetCycle();
  void LogStatus() const;
  double TotalStep() const;

private:
  void TwiddleParams(double error);
};
