#include <fstream>
#include <iostream>
#include <chrono>
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

  double last_cte;

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
