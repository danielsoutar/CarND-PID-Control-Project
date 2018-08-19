#include <iostream>
#include <math.h> 

#include "PID.h"


PID::PID() {
	K_coeffs[0] = K_coeffs[1] = K_coeffs[2] = 0.0;
    errors[0] = errors[1] = errors[2] = 0.0;
}

PID::PID(double Kp, double Kd, double Ki) {
    K_coeffs[0] = Kp;
    K_coeffs[1] = Kd;
    K_coeffs[2] = Ki;
    errors[0] = errors[1] = errors[2] = 0.0;
}

PID::~PID() {}


void PID::Init(double init_CTE) {
    errors[0] = init_CTE;
    errors[1] = errors[2] = 0.0;
    prev_CTE = init_CTE;
    total_error = 0.0;
    record_error = false;
}

void PID::SetParams(int index, double value) {
    K_coeffs[index] = value;
}

void PID::UpdateParams(int index, double value) {
    K_coeffs[index] += value;
}

void PID::ResetParams() {
    for (int i = 0; i < NUM_PARAMS; ++i)
        K_coeffs[i] = 0.0;
}

void PID::PrintParams() {
    for (int i = 0; i < NUM_PARAMS; ++i)
        std::cout << "P" << i << ": " << K_coeffs[i] << ", ";
    std::cout << "\n";
}

void PID::ResetErrors() {
    for (int i = 0; i < NUM_PARAMS; ++i)
        errors[i] = 0.0;
}

double PID::Respond(double CTE) {
    errors[0] = CTE;
    errors[1] = CTE - prev_CTE;
    prev_CTE = CTE;
    errors[2] += CTE; 

    double alpha = -(K_coeffs[0] * errors[0]) - (K_coeffs[1] * errors[1]) - (K_coeffs[2] * errors[2]);

    if(record_error)
        total_error += pow(CTE, 2);

    // Smooth out extreme values of alpha by imposing a maximum angle.
    if(alpha > 1.0)
    	alpha = 1.0;
    else if(alpha < -1.0)
    	alpha = -1.0;

    return alpha;
}

double PID::RespondThrottle(double angle, double max_velocity) {
    errors[0] = angle;
    errors[1] = angle - prev_angle;
    prev_angle = angle;
    errors[2] += angle;

    double throttle = max_velocity - (K_coeffs[0] * errors[0]) - (K_coeffs[1] * errors[1]) - (K_coeffs[2] * errors[2]);

    if(record_error)
        total_error += pow(angle, 2);

    // No need to limit the variable here, already did that by imposing max_velocity.
    return throttle;
}

void PID::RecordTotalError() {
    record_error = true;
}

double PID::GetTotalError() {
    return total_error;
}
