//
//  twiddle.cpp
//  PID
//
//  Created by David Moriano on 26/05/2017.
//
//

#include "twiddle.h"
#include <iostream>
#include <iomanip>

std::ostream& operator<<(std::ostream& o, const std::valarray<double> &array) {
  o << "[";
  for (int i = 0; i < array.size(); ++i) {
    if (i > 0) o << ", ";
    o << array[i];
  }
  o << "]";
  return o;
}

void Twiddle::Init(const array& params, const array& steps, const int cycle)
{
  this->params = params;
  this->steps = steps;
  this->cycle_size = cycle;

  this->best_error = INFINITY;
  this->current_error = 0;
  this->current_param = 0;
  this->count = 0;
  this->state = 0;
}

void Twiddle::ResetCycle()
{
  std::cout << "Twiddle: ResetCycle" << std::endl;
  // reset param back
  if (state == 1) {
    params[current_param] -= 2 * steps[current_param];
  } else if (state == 2) {
    params[current_param] += steps[current_param];
  }
  // try next param
  current_param = (current_param + 1) % params.size();
  // Reset state
  current_error = 0;
  state = 0;
  count = 0;
}


bool Twiddle::Update(double error)
{
  count ++;
  current_error += error;

  LogStatus();

  if (count == cycle_size) {
    // Simulation cycle completed, get error and twiddle params
    TwiddleParams(current_error / cycle_size);
    // reset error
    current_error = 0;
    // reset count
    count = 0;
    // must update parameters in system
    return true;
  }

  return false;
}

double Twiddle::TotalStep() const
{
  return std::abs(steps).sum();
}

void Twiddle::TwiddleParams(double error)
{
  if (state == 0) {
    // ascend
    params[current_param] += steps[current_param];
    state = 1; // simulate
    return;
  }

  if (error < best_error) {
    // success, keep going
    best_error = error;
    best_params = params;
    best_steps = steps;
    steps[current_param] *= (1.0 + delta);
  }
  else if (state == 1) {
    // failed, descent and keep trying
    params[current_param] -= 2 * steps[current_param];
    state = 2; // simulate
    return;
  }
  else { // state == 2
    // ascend and descent failed, recover original and shrink
    params[current_param] += steps[current_param];
    steps[current_param] *= (1.0 - delta);
  }
  
  // switch to next param
  current_param = (current_param + 1) % params.size();
  // ascend
  params[current_param] += steps[current_param];
  state = 1; // simulate
}

void Twiddle::LogStatus() const
{
  std::ios::fmtflags flags(std::cout.flags());
  std::cout << std::scientific << std::setprecision(3);

  std::cout << "[" << count << "] Twiddle:";

  std::cout << " p = ";
  for(auto p : params)
    std::cout << p << " ";

  std::cout << " dp = ";
  for(auto dp : steps)
    std::cout << dp << " ";

  std::cout << std::fixed;
  std::cout << " error: " << (current_error / count)
            << " best_error: " << best_error
            << " best_params: " << best_params
            << " step: " << TotalStep()
            << " param: " << current_param
            << " state: " << state
            << " cycle: " << count <<  " of " << cycle_size
            << std::endl;

  std::cout.flags(flags);
}
