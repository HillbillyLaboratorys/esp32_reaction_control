#include <stdio.h>
#include <math.h>
#include "i2c.h"
#include "driver\gpio.h"
#include "freeRTOS\freeRTOS.h"
#include "freeRTOS\task.h"
#include "esp_log.h"
#include "driver\uart.h"

#define BUILTIN_LED GPIO_NUM_2



// static void uart_init()
// {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    // uart_config_t uart_config = {
    //     .baud_rate = 115200,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity    = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .source_clk = UART_SCLK_DEFAULT,
    // };
    // ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0));
//}
static const char *TAG = "acc_data";
void app_main(void)
{
    gpio_reset_pin(BUILTIN_LED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BUILTIN_LED, GPIO_MODE_OUTPUT);

    //uart_init();
    i2c_master_init();
    //ESP_ERROR_CHECK()

    uint8_t data[6];
    //uart_write_bytes(UART_NUM_0, data, 1024);
    
    // reset mem
    //uint8_t ctl[1];
    //ctl[0] = 0b10000100;
    write_to_lsm6dsl(ACC_GRO_CTRL3, 0b10000100);

    vTaskDelay(100);

    // Enable ACC 208Hz data rate    
    //read_registers(ACC_CTRL1_XL, ctl, 1);
    //ESP_LOGI(TAG, "ctl_1: %d", ctl[0]);
    //ctl[0] &= ~0xF0; // clear 4 MSBs
    //ctl[0] = 0b01100011; 
    write_to_lsm6dsl(ACC_CTRL1_XL, 0b01101011);

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
    write_to_lsm6dsl(CTRL2_G, 0b01101100);

    int16_t a[3];
    int16_t g[3];
    float magX, magY, magZ;
    float aAngX, aAngY;
    float GX, GY, GZ;
    uint32_t on = 0;
    //float temp = 0;
    float ares = 4.0 / 32768.0; // max 16 bit 65536/2 (for negitive) = 32768
    float gres = 2000.0 / 32768.0;
    float pi =  3.14159265358979323846;
    float dpr = 57.29578;

    while(1)
    {
        gpio_set_level(BUILTIN_LED, 1);
        vTaskDelay(2);
        gpio_set_level(BUILTIN_LED, 0);
        read_from_lsm6dsl(OUTX_L_XL, data, 6);

        a[0] = (int16_t)(data[0] | data[1] << 8);
        a[1] = (int16_t)(data[2] | data[3] << 8);
        a[2] = (int16_t)(data[4] | data[5] << 8);
        
        magX = ((float)a[0]) * ares;
        magY = ((float)a[1]) * ares;
        magZ = ((float)a[2]) * ares;

        aAngX = (float)(atan2(magY,magZ)+pi) * dpr;
        aAngY = (float)(atan2(magY,magX)+pi) * dpr;

        vTaskDelay(5);
        //temp = atan2(g[1],g[2]);
        read_from_lsm6dsl(OUTX_L_G, data, 6);

        g[0] = (int16_t)(data[0] | data[1] << 8);
        g[1] = (int16_t)(data[2] | data[3] << 8);
        g[2] = (int16_t)(data[4] | data[5] << 8);
        
        //g[0] *= 0.07

        GX = ((float)g[0]) * gres;

        GY = ((float)g[1]) * gres;

        GZ = ((float)g[2]) * gres;

        ESP_LOGI(TAG, "%f %f %f  %f %f  G: %f %f %f\n\r", magX, magY, magZ, aAngX, aAngY, GX, GY, GZ);
        
        //uart_write_bytes(UART_NUM_0, g, 3);
        vTaskDelay(40);
        on = !on;
    }

}

