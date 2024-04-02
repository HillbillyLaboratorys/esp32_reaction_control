#include "driver/mcpwm_prelude.h"

#define MIN_PULSEWIDTH 100 // in us 1000
#define MAX_PULSEWIDTH 15000 // 2000
#define ESC_RESOLUTION 1000000 // cycles per tick
#define ESC_TICKS_PER_PERIOD 20000 // 20 us
#define ESC_GPIO 2

typedef struct 
{
    mcpwm_oper_handle_t operator;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    mcpwm_timer_handle_t timer;
} esc_pwm;

esc_pwm esc_init(void);
void esc_speed_control(mcpwm_cmpr_handle_t comparator, float _percent);

