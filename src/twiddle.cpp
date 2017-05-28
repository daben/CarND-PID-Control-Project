//
//  twiddle.cpp
//  PID
//
//  Created by David Moriano on 26/05/2017.
//
//

#include "twiddle.h"


void Twiddle::Init(const array& params, const array& perturbation, const int cycle)
{
  this->params = params;
  this->perturbation.resize(params.size());
  this->perturbation = perturbation;
  
  this->count_cycle = cycle;
  this->count = 0;
  this->current_error = 0;
  this->best_error = -1;
  this->current_param = 0;
  this->_state = 0;
}

void Twiddle::Reset()
{
  count = 0;
  current_error = 0;
  _state = 0;
}

bool Twiddle::Update(double cte)
{
  current_error += cte * cte;
  count ++;
  
  if (count % count_cycle == 0) {
    // Simulation cycle completed, get error and twiddle params
    TwiddleParams();
    // reset error
    current_error = 0;
    // must update parameters in system
    return true;
  }
  
  return false;
}

double Twiddle::TotalPerturbation()
{
  return std::abs(perturbation).sum();
}

void Twiddle::TwiddleParams()
{
  do
  {
    switch (_state) {
      case 0:
        best_error = current_error;
//        printf("params[%d] += %.2f\n", current_param, perturbation[current_param]);
        params[current_param] += perturbation[current_param];
        _state = 1; // simulate
        break;
        
      case 1:
        if (current_error < best_error) {
          best_error = current_error;
          perturbation[current_param] *= 1.05;
          // finished with this param
          current_param = (current_param + 1) % params.size();
          _state = 0;
        } else {
          params[current_param] -= 2 * perturbation[current_param];
          _state = 2; // simulate
        }
        break;
        
      case 2:
        if (current_error < best_error) {
          best_error = current_error;
          perturbation[current_param] *= 1.05;
        } else {
          params[current_param] += perturbation[current_param];
//          printf("params[%d] += %.2f\n", current_param, perturbation[current_param]);
          perturbation[current_param] *= 0.95;
        }
        
        // finished with this param
        current_param = (current_param + 1) % params.size();
        _state = 0;
        break;
    }
  } while (_state == 0);
}

