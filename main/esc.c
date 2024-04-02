#include "esc.h"

esc_pwm esc_init(void) {
    
    // Timer
    mcpwm_timer_handle_t timer_hndl = NULL;
    mcpwm_timer_config_t timer_conf = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = ESC_RESOLUTION,
        .period_ticks = ESC_TICKS_PER_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };    
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_conf, &timer_hndl));

    // Operator
    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_conf = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_conf, &oper));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer_hndl)); 

    // Comparator
    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    // Generator
    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = ESC_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    esc_speed_control(comparator, 0);

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer_hndl));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer_hndl, MCPWM_TIMER_START_NO_STOP));

    esc_pwm esc = {
        .timer = timer_hndl,
        .operator = oper,
        .comparator = comparator,
        .generator = generator,
    };

    return esc;
}


/* Takes percent (0-1) and writes perportional pwm signal to esc
*
*/
void esc_speed_control(mcpwm_cmpr_handle_t comparator, float _percent) {
    uint32_t pulse_w = (MIN_PULSEWIDTH + ((MAX_PULSEWIDTH - MIN_PULSEWIDTH) * _percent));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_w));
}