#include "PID.h"
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  p_error = d_error = i_error = 0.;

  stepNum = 1;
  numEvalSteps = 2000;
  numSettleSteps = 100;

  twiddleVals[0] = Kp*0.1;
  twiddleVals[1] = Kd*0.1;
  twiddleVals[2] = Ki*0.1;

  best_error = std::numeric_limits<double>::max();
  paramIndex = 2;  // To start from 1st proportional-parameter after the first Twiddle.
}

void PID::UpdateError(double cte) {
  if (stepNum == 1) {  // Set the first P-error to CTE.
    p_error = cte;
  }
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  // Update total-error after completing a number of settle-steps steps.
  if (stepNum % (numEvalSteps + numSettleSteps) > numSettleSteps) {
    total_error += cte*cte;
  }

  // Run Twiddle optimisation routine, for every 'n' evaluation steps.
  if (stepNum % (numEvalSteps + numSettleSteps) == 0 && optimiseWithTwiddle) {
    cout << "Step number: " << stepNum << "\n";
    cout << "Total error: " << total_error << "\n";
    cout << "Best error: " << best_error << "\n";
    if (total_error < best_error) {
      cout << "Optimised!!\n";
      best_error = total_error;
      // First twiddle-loop with paramIndex set to 2
      if (stepNum != numEvalSteps + numSettleSteps) {
        twiddleVals[paramIndex] *= 1.1;
      }
      // Next parameter to optimise.
      paramIndex = (paramIndex + 1) % 3;
      paramAdded = paramSubtracted = false;
    }
    if (!paramAdded && !paramSubtracted) {
      // Twiddle-add
      TwiddleParameter(paramIndex, twiddleVals[paramIndex]);
      paramAdded = true;
    }
    else if (paramAdded && !paramSubtracted) {
      // Twiddle-subtract
      TwiddleParameter(paramIndex, -2*twiddleVals[paramIndex]);
      paramSubtracted = true;
    }
    else {
      // Go back to original state and reduce twiddle-value
      TwiddleParameter(paramIndex, twiddleVals[paramIndex]);
      twiddleVals[paramIndex] *= 0.9;
      paramAdded = paramSubtracted = false;
      // Switch to next parameter.
      paramIndex = (paramIndex + 1) % 3;
    }
    total_error = 0;
    cout << "Adjusted parameters ..." << "\n";
    cout << "Kp = " << Kp << " Ki = " << Ki << " Kd = " << Kd << "\n\n";
  }
  ++stepNum;
}

double PID::TotalError() {
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

void PID::TwiddleParameter(int paramIndex, double value) {
  switch(paramIndex) {
    case 0:  // Proportional
      Kp += value;
      break;
    case 1:  // Derivative
      Kd += value;
      break;
    case 2:  // Integral
      Ki += value;
      break;
    default:
      break;
  }
}