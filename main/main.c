#include <stdio.h>
#include <math.h>
#include "i2c.h"
#include "esc.h"
#include "driver\gpio.h"
#include "freeRTOS\freeRTOS.h"
#include "freeRTOS\task.h"
#include "esp_log.h"
#include "driver\uart.h"
#include "esp_timer.h"
#include "pid.h"

#define CAL_ITER 20 // num samples to calibrate gyro

#define DT 0.01
#define PER DT * 1000000 //us

#define FILTER_CONST 0.8
#define TARGET_ANGLE 0
// no need for Acc resolution beacause we use a ratio

//#define CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD

static void run_pid_callback(void* arg);

#define BUILTIN_LED GPIO_NUM_2

static const char *TAG = "IMU_data";
void app_main(void)
{
    uint32_t prevspeed1 = 0;
    int16_t a[3];
    int16_t g[3];
    float magX, magY, magZ;
    float aAngX, aAngY;
    float dpr = 57.29578;
    uint8_t data[12];
    int16_t tg_cal[3];
    float g_cal[3];
    uint8_t led_s = 0;
    float gx_ang = 0;
    float gy_ang = 0;
    float gz_ang = 0;
    float GRES = 2000.0 / 32768.0; // max 16 bit 65536/2 (for +/-) = 32768, 2000 dps

    float x_filt, y_filt = 0;

    //float gx_cal, gy_cal, gz_cal = 0;

    PIDParams x_pid_param = {
        .kp = 10,
        .ki = 0.0,
        .kd = 0.01
    };

    double pid_out = 0.0;

    esc_pwm esc1 = esc_init();
    
    //ledc_timer_pause(LEDC_MODE, LEDC_TIM);    

    //ledc_set_freq(LEDC_HIGH_SPEED_MODE, LEDC_TIMER_0, 10000);

    gpio_reset_pin(BUILTIN_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BUILTIN_LED, GPIO_MODE_OUTPUT);

    //uart_init();
    i2c_master_init();
    //ESP_ERROR_CHECK()

    //uart_write_bytes(UART_NUM_0, data, 1024);
    
    // reset mem
    //uint8_t ctl[1];
    //ctl[0] = 0b10000100;
    
    //vTaskDelay(10);

    // Enable ACC 208Hz data rate    
    //read_registers(ACC_CTRL1_XL, ctl, 1);
    //ESP_LOGI(TAG, "ctl_1: %d", ctl[0]);
    //ctl[0] &= ~0xF0; // clear 4 MSBs
    //ctl[0] = 0b01100011; 

    write_to_lsm6dsl(CTRL2_G, 0b01101100); // 0b10011100 gyro 3.33 kHz, 2000 dps, 125dps disabled

    write_to_lsm6dsl(ACC_CTRL1_XL, 0b01000111); //0b10011011 ACC 3.33 kHz, 4G, ODR/4, 400Hz band

    write_to_lsm6dsl(ACC_GRO_CTRL3, 0b01000100); // block update + auto inc 

    //ctl[0] = 0b11001000;
    //wright_regester(CTRL8_XL, ctl, 1);

    //ctl[0] = 0b01000100;
    //wright_regester(ACC_GRO_CTRL3, ctl, 1);
    //ESP_LOGI(TAG, "ctl_1 w: %d", ctl[0]);

    //use default scale of +-2g

    // CTRL4_C 0x80 DEN_XLEN???
    //ctl[0] = 0b10000000;
    //wright_regester(0x13, ctl, 1);

    //read_registers(CTRL2_G, ctl, 1);
    //ctl[0] = 0b01101100; // 3.3kHz 2000 dps
    
    //write_to_lsm6dsl(CTRL7_G, 0b01010000); // enable gyro high pass at 65mHz

    vTaskDelay(5);

    // Drift Cal
    for (uint8_t i = 0; i < CAL_ITER; ++i) {
        read_from_lsm6dsl(OUTX_L_G, data, 6);

        tg_cal[0] += (int16_t)(data[0] | data[1] << 8);
        tg_cal[1] += (int16_t)(data[2] | data[3] << 8);
        tg_cal[2] += (int16_t)(data[4] | data[5] << 8);
        led_s = ~led_s;
        gpio_set_level(BUILTIN_LED, led_s);
        vTaskDelay(10);
    }
    g_cal[0] = (float)tg_cal[0] / (float)CAL_ITER;
    g_cal[1] = (float)tg_cal[1] / (float)CAL_ITER;
    g_cal[2] = (float)tg_cal[2] / (float)CAL_ITER;
    gpio_set_level(BUILTIN_LED, 1);
  
    uint8_t temp = 0;
    uint8_t * run = &temp;

    const esp_timer_create_args_t pid_args = {
        .dispatch_method = ESP_TIMER_ISR,
        .arg = (void*)run,
        .callback = &run_pid_callback,
        .name = "cycleTimer",
        .skip_unhandled_events = true,
    };

    uint32_t prev_time = 0;
    uint32_t cur_time = 0;
    float dt = 0;
    //uint32_t enable_step = 0;

    esp_timer_handle_t cycle_timer;
    ESP_ERROR_CHECK(esp_timer_create(&pid_args, &cycle_timer));

    ESP_ERROR_CHECK(esp_timer_start_periodic(cycle_timer, PER)); 


    while(1) {
        if (*run) {

            // esc_speed_control(esc.comparator, 0.9);
            // vTaskDelay(40);
            // esc_speed_control(esc.comparator, 0.0);
            // vTaskDelay(40);
            // esc_speed_control(esc.comparator, 0.2);
            // vTaskDelay(2000);
            // esc_speed_control(esc.comparator, 0.1);

            
            //vTaskDelay(2);
            // gpio_set_level(BUILTIN_LED, 0);
            
            prev_time = cur_time;

            cur_time = esp_timer_get_time();

            dt = ((float)cur_time - (float)prev_time)/1000000; //0.01; //(float)(cur_time - prev_time) / 1000000.0; // PWM Timer Init causing large inital value

            read_from_lsm6dsl(OUTX_L_G, data, 12);
            

            a[0] = ((int16_t)data[6]) | data[7] << 8;
            a[1] = ((int16_t)data[8]) | data[9] << 8;
            a[2] = ((int16_t)data[10]) | data[11] << 8;
            
            magX = ((float)a[0]); //* ares;
            magY = ((float)a[1]); //* ares;
            magZ = ((float)a[2]); //* ares;
            
            aAngX = (float)(atan2f(magY,magZ)) * dpr;
            aAngY = (float)(atan2f(magX,magZ)) * dpr;
            
            //vTaskDelay(5);
            //temp = atan2(g[1],g[2]);
            //read_from_lsm6dsl(OUTX_L_G, data, 6);

            g[0] = ((int16_t)data[0]) | data[1] << 8;
            g[1] = ((int16_t)data[2]) | data[3] << 8;
            g[2] = ((int16_t)data[4]) | data[5] << 8;
           
            gx_ang += (((float)g[0]) * GRES) * dt; //- g_cal[0]

            gy_ang += ((float)g[1]) * GRES * dt; 

            gz_ang += ((float)g[2]) * GRES * dt; 

            x_filt = (FILTER_CONST * gx_ang) + ((1-FILTER_CONST) * aAngX);
            y_filt = (FILTER_CONST * gy_ang) + ((1-FILTER_CONST) * aAngY);

            //esc_speed_control(esc.comparator, (x_filt+90.0)/180.0);
            
            // Run PID with new current position and time
            pid_out = PID_Compute(&x_pid_param, TARGET_ANGLE, x_filt, dt);

            // Send PID output to ESC
            prevspeed1 = esc_accel_control(esc1.comparator, pid_out + prevspeed1);

            // Enables Stepper on first iteration
            // if (!enable_step) {
            //     ledc_timer_resume(LEDC_MODE, LEDC_TIM);
            //     enable_step = 1;
            // }

            ESP_LOGI(TAG, "%f %f  G: %f %f %f PID %f freq %i comp %f\n\r", x_filt, y_filt, gx_ang, gy_ang, dt, pid_out, (int)prevspeed1, g_cal[0]
            );

            *run = 0;
        }
    }
}
    void run_pid_callback(void* arg) {
        *((uint8_t*)arg) = 1;
    }



