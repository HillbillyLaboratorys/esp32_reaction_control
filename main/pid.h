// PID parameters
struct PIDParams;

typedef struct PIDParams {
    float kp;
    float ki;
    float kd;
} PIDParams;
 
// Function declarations
// void PID_Init(PIDParams* params, double kp, double ki, double kd);
float PID_Compute(PIDParams* params, float setpoint, float actual_value, float dt);
void PID_Reset(PIDParams* params);