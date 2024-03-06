#include <stdio.h>
#include "i2c.h"
#include "driver\gpio.h"
#include "freeRTOS\freeRTOS.h"
#include "freeRTOS\task.h"
#include "driver\uart.h"

#define BUILTIN_LED GPIO_NUM_2

static void uart_init()
{
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0));
}

void app_main(void)
{

    uart_init();
    i2c_master_init();
    //ESP_ERROR_CHECK()

    uint8_t data[1024] = "test\n\r";
    uart_write_bytes(UART_NUM_0, data, 1024);
    
    // Enable Block Data Update
    uint8_t ctl[1];
    read_from_lsm6dsl(ACC_GRO_CTRL3, ctl, 1);
    ctl[0] |= 64;
    write_to_lsm6dsl(ACC_GRO_CTRL3, *ctl);

    // Enable ACC 208Hz data rate    
    read_from_lsm6dsl(ACC_CTRL1_XL, ctl, 1);
    ctl[0] &= ~0xF0; // clear 4 MSBs
    ctl[0] |= 0x50; 
    write_to_lsm6dsl(ACC_CTRL1_XL, *ctl);

    //use default scale of +-2g


    while(1)
    {

        read_from_lsm6dsl(OUTX_L_XL, data, 6);
        uart_write_bytes(UART_NUM_0, data, 6);
        vTaskDelay(50);
    
    }

}

