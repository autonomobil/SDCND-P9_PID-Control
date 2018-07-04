#include "PID.h"

using namespace std;


PID::PID() {}

PID::~PID() {}



void PID::Init(double Kp, double Ki, double Kd) {
	p_error = 0;
	i_error = 0;
	d_error = 0;
	last_cte = 0;


	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	cout << "Kp: " << Kp  << '\t' << "Ki: " << Ki << '\t'  << "Kd: " << Kd << endl;

	m_timeBegin = std::chrono::steady_clock::now( );
}

void PID::UpdateError(double cte) {

	p_error = cte;

	i_error += cte;

	d_error = cte - last_cte;

	last_cte = cte;

}



double PID::TotalError(double speed) {

	double steering_angle = -Kp_ * p_error - Ki_ * i_error  - Kd_  * d_error ;

	if(steering_angle > 0.9){
		steering_angle = 0.9;

	}
	else if(steering_angle < -0.9){
		steering_angle = -0.9;
	}

	return steering_angle;
}

long int PID::Time_stepper(){

	auto timeNow = std::chrono::steady_clock::now();
	auto msSinceCreation = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - m_timeBegin);

	return msSinceCreation.count( );

}
