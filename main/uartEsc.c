#include "uartEsc.h"

uart_port_t uartEscinit() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    return UART_NUM_2;
}

void sendUartEsc(float voltage) {
    if (voltage > 12) voltage = 12;
    if (voltage < -12) voltage = -12;
    char out [20];
    int l = sprintf(out, "T%f\n", voltage);
    uart_write_bytes(UART_NUM_2, (const char*)out, l);
    uart_write_bytes(UART_NUM_2, "test", 6);
}