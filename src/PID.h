#include <iomanip>
#include <iostream>
#include <chrono>
#include <math.h>
#ifndef PID_H
#define PID_H

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  // d_error accumalated
  double d_error_acc;
  double last_cte;

  // steering value accumalated
  double steering_val_acc;

  // clock
  chrono::time_point<chrono::steady_clock> m_timeBegin;

  /*
  * Coefficients
  */
  double Kp_;
  double Ki_;
  double Kd_;

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
  double TotalError(double speed);

  /*
  * Count and output timestep
  */
  long int Time_stepper();

};

#endif /* PID_H */
