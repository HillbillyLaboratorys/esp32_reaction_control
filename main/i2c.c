#include "i2c.h"

esp_err_t i2c_master_init(void) {

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_CLK, 
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(I2C_PORT, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}


esp_err_t i2c_write_to_bus(uint8_t dev_add, uint8_t* data, size_t len) {

    i2c_cmd_handle_t _cmd_hndl = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(_cmd_hndl);
    if (err != ESP_OK) {
        return err;
    }
    i2c_master_write_byte(_cmd_hndl, (dev_add << 1) | I2C_MASTER_WRITE, true); //read write indication is embedded in the slave adderess
    i2c_master_write(_cmd_hndl, data, len, true);
    i2c_master_stop(_cmd_hndl);
    err = i2c_master_cmd_begin(I2C_PORT, _cmd_hndl, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(_cmd_hndl);
    return err;
}

esp_err_t i2c_read_from_bus(uint8_t dev_add, uint8_t* data, size_t len) {

    if (len == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_add << 1) | I2C_MASTER_READ, true); 
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, 0);
    }
    i2c_master_read_byte(cmd, data + len - 1, 1); // master indicates it's done reading with a NACK
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t write_to_lsm6dsl(uint8_t sub_add, uint8_t _command) {
    //i2c_master_write_to_device()
     i2c_cmd_handle_t _cmd_hndl = i2c_cmd_link_create();
    esp_err_t err = i2c_master_start(_cmd_hndl);
    if (err != ESP_OK) {
        return err;
    }
    i2c_master_write_byte(_cmd_hndl, (I2C_ACC_ADD << 1) | I2C_MASTER_WRITE, true); //read write indication is embedded in the slave adderess
    i2c_master_write_byte(_cmd_hndl, sub_add, true);
    i2c_master_write_byte(_cmd_hndl, _command, true);
    i2c_master_stop(_cmd_hndl);
    err = i2c_master_cmd_begin(I2C_PORT, _cmd_hndl, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(_cmd_hndl);
    return err;
}

esp_err_t read_from_lsm6dsl(uint8_t sub_add, uint8_t* data, size_t len) {

     if (len == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ACC_ADD << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, sub_add, true); 

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ACC_ADD << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, 0);
    }
    i2c_master_read_byte(cmd, data + len - 1, 1); // master indicates it's done reading with a NACK
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

void read_registers(uint8_t reg_addr, uint8_t* data, uint8_t len) {
        
    i2c_master_write_read_device(I2C_PORT, I2C_ACC_ADD, &reg_addr, 1, data, len, pdMS_TO_TICKS(1000));

}

void wright_regester(uint8_t reg_addr, uint8_t* data, uint8_t len) {

    i2c_master_write_to_device(I2C_PORT, I2C_ACC_ADD, data, len, pdMS_TO_TICKS(1000));
}