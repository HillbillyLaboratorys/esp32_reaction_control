#include "driver\i2c.h"

#define I2C_PORT I2C_NUM_0
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_CLK 100000

#define I2C_ACC_ADD 0x6A // or 0x6A

#define ACC_GRO_CTRL3 0x12
#define CTRL8_XL 0x17
#define CTRL2_G 0x11
#define CTRL7_G 0x16
#define ACC_CTRL1_XL 0x10 // linear accel
#define OUTX_L_XL 0x28
#define OUTX_L_G 0x22

esp_err_t i2c_master_init(void);

esp_err_t i2c_write_to_bus(uint8_t dev_add, uint8_t* data, size_t len);

esp_err_t i2c_read_from_bus(uint8_t dev_add, uint8_t* data, size_t len);

esp_err_t write_to_lsm6dsl(uint8_t sub_add, uint8_t _command);

esp_err_t read_from_lsm6dsl(uint8_t sub_add, uint8_t* data, size_t len);

void read_registers(uint8_t reg_addr, uint8_t* data, uint8_t len );

void wright_regester(uint8_t reg_addr, uint8_t* data, uint8_t len);