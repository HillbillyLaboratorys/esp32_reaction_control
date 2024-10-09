// PID parameters
struct PIDParams;

typedef struct PIDParams {
    double kp;
    double ki;
    double kd;
} PIDParams;
 
// Function declarations
// void PID_Init(PIDParams* params, double kp, double ki, double kd);
double PID_Compute(PIDParams* params, double setpoint, double actual_value, double dt);
void PID_Reset(PIDParams* params);