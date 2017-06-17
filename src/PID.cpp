#include "PID.h"
#include <iostream>

using namespace std;

class Twiddler {

public:
	double tW[3];
	int index;
	int sign;
	double best_error;

	Twiddler(double tP, double tI, double tD);

	void twiddle(PID& pid, double error);

};

Twiddler::Twiddler(double tP, double tI, double tD) {
	index = -1;
	tW[0] = tP;
	tW[1] = tI;
	tW[2] = tD;
	sign = 0;
	best_error = 0;
}

void Twiddler::twiddle(PID& pid, double error) {

	double* p[3] = { &pid.Kp, &pid.Ki, &pid.Kd };

	if (index == -1) {
		index = std::rand() % 3;
		best_error = error;
	} else {
		if (error < best_error) {
			sign = 0;
			best_error = error;
			tW[index] *= 1.1;
			index = std::rand() % 3;
		} else {
			switch (sign) {
				case 0:
					*p[index] += tW[index];
					sign++;
					break;
				case 1:
					*p[index] -= 2 * tW[index];
					sign++;
					break;
				case 2:
					*p[index] += tW[index];
					tW[index] *= 0.9;
					index = std::rand() % 3;
					sign = 0;
					break;
			}
		}
	}

	std::cout << " Kp " << pid.Kp << ", Ki " << pid.Ki << ", Kd " << pid.Kd << std::endl;
	std::cout << " tW[0] " << tW[0] << ", tW[1] " << tW[1] << ", tW[2] " << tW[2] << std::endl;

}

/*
* TODO: Complete the PID class.
*/

const double MAX_INT_FRAMES = 10;

static Twiddler twiddler(0.05, 0.001, 0.25);

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	this->d_error = 0;
	this->i_error = 0;
	this->p_error = 0;
}

void PID::UpdateError(double cte) {
	//if (cte > 0.75 || cte < -0.75)
	//	twiddler.twiddle(*this, cte);

	if (this->d_error == 0 && this->p_error == 0)
		this->d_error = 0;
	else
		this->d_error = cte - this->p_error;
	this->p_error = cte;
	this->i_error += cte;
	//subtract the mean value from total per frame, this limits the
	this->i_error -= this->i_error / MAX_INT_FRAMES;
	//std::cout << "p_error " << this->p_error << ", i_error " << this->i_error << ", d_error " << this->d_error << std::endl;
}

double PID::TotalError() {
	double error = - this->Kp * this->p_error - this->Kd * this->d_error - this->Ki * this->i_error;
	return error;
}

