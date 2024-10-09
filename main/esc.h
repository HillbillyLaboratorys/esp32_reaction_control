#include "driver/mcpwm_prelude.h"
#include "driver/ledc.h"

#define MIN_FREQ 400 
#define MAX_FREQ 1500
// #define ESC_RESOLUTION 1000000 // cycles per tick
// #define ESC_TICKS_PER_PERIOD 20000 // 20 us
#define ESC_GPIO 19
#define DIRECTION_GPIO 18

#define LEDC_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_CHAN LEDC_CHANNEL_0
#define LEDC_TIM LEDC_TIMER_0
#define LEDC_DUTY 64 // 256 (8 bit res)
#define LEDC_FREQUENCY 1000
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT

// typedef struct 
// {
//     mcpwm_oper_handle_t operator;
//     mcpwm_cmpr_handle_t comparator;
//     mcpwm_gen_handle_t generator;
//     mcpwm_timer_handle_t timer;
// } esc_pwm;

void esc_init(void);
uint32_t esc_accel_control(ledc_mode_t speed_mode, ledc_timer_t timer_num, double speed);

