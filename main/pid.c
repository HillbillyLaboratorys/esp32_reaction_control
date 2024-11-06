#include "PID.h"


// PID state variables
static double prev_error = 0.0;
static double integral = 0.0;

// Initialize the PID parameters
// void PID_Init(PIDParams* params, double kp, double ki, double kd) {
//     params->kp = kp;
//     params->ki = ki;
//     params->kd = kd;
//     prev_error = 0.0;
//     integral = 0.0;
// }

// Compute the PID output based on the setpoint, actual value, and time delta
float PID_Compute(PIDParams* params, float setpoint, float actual_value, float dt) {
    // Calculate error
    float error = setpoint - actual_value;

    // Proportional term
    float p_term = params->kp * error;

    // Integral term (sum of errors)
    integral += error * dt;
    float i_term = params->ki * integral;

    // Derivative term (change in error)
    float derivative = (error - prev_error) / dt;
    float d_term = params->kd * derivative;

    // Store the current error for the next iteration
    prev_error = error;

    // Return the sum of PID terms
    return p_term + i_term + d_term;
}

// Reset the PID controller (clears the integral and previous error)
void PID_Reset(PIDParams* params) {
    prev_error = 0.0;
    integral = 0.0;
}
// the epic of gilgamesh
