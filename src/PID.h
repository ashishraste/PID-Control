#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double total_error;
  double best_error;

  unsigned int stepNum;
  unsigned int numEvalSteps;
  unsigned int numSettleSteps;
  double twiddleVals[3];
  int paramIndex;
  bool paramAdded;
  bool paramSubtracted;
  bool optimiseWithTwiddle;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
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

  void TwiddleParameter(int paramIndex, double value);

};

#endif /* PID_H */
