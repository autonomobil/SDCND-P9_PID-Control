#include "PID.h"

using namespace std;


PID::PID() {}

PID::~PID() {}



void PID::Init(double Kp, double Ki, double Kd) {
	p_error = 0;
	i_error = 0;
	d_error = 0;
	steering_val_acc = 0;
	d_error_acc = 0;
	last_cte = 0;

	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	m_timeBegin = std::chrono::steady_clock::now( );
}

void PID::UpdateError(double cte) {

	p_error = cte;

	if(fabs(cte) < 0.01){
		i_error = 0;
	}
	else{
		i_error += cte;

		if(i_error > 50.0){
			i_error = 50.0;
		}
		else if(i_error < -50.0){
			i_error = -50.0;
		}
	}

	// smooth d_error
	d_error_acc = (0.5 * (cte - last_cte)) + (1.0 - 0.5) * d_error_acc;
	d_error = d_error_acc ;

	last_cte = cte;
}


double PID::TotalError(double speed) {
	cout << endl;
	double p_val = -(Kp_ - speed/500) * p_error;
	double i_val = -Ki_ * i_error;
	double d_val = -(Kd_ + speed/50) * d_error;

	cout << "speed:" << setw(1) << speed <<
	        setw(15) << "p value:" << setw(2)  << p_val <<
					setw(15) << "i value:" << setw(2)  << i_val <<
					setw(15) << "d value:" << setw(2)  << d_val << '\n';

	double steering_val = p_val + i_val + d_val;

	if(steering_val > 0.95){
		steering_val = 0.95;
	}
	else if(steering_val < -0.95){
		steering_val = -0.95;
	}

	// smooth steering_val
	steering_val_acc = (0.6 * steering_val) + (1.0 - 0.6) * steering_val_acc;
	steering_val = steering_val_acc;

	return steering_val;
}

long int PID::Time_stepper(){

	auto timeNow = std::chrono::steady_clock::now();
	auto msSinceCreation = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - m_timeBegin);

	return msSinceCreation.count( );

}
