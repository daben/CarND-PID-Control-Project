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
public:
  using array = std::valarray<double>;

  array params;
  array perturbation;
  
// private:
  int count_cycle;
  int count;
  double current_error;
  double best_error;
  int current_param;
  int _state;

public:
  Twiddle() {}
  ~Twiddle() {}
  
  void Init(const array& params, const array& perturbation, const int cycle);
  
  bool Update(double cte);
 
  double TotalPerturbation();
  
  void Reset();
  
private:
  void TwiddleParams();
};
