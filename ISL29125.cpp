#include <stdio.h>
#include "pico/stdlib.h"
#include "ISL29125.h"

ISL29125::ISL29125(i2c_inst_t *i2c_port, uint sda_pin, uint scl_pin, uint i2c_freq, mutex_t *i2c_mutex)
: i2c_port_(i2c_port), sda_pin_(sda_pin), scl_pin_(scl_pin), i2c_freq_(i2c_freq), i2c_mutex_(i2c_mutex) {}

bool ISL29125::init_sensor(RGB_OP_MODES mode, SENS_RANGE range, RES res, bool ir_offset, IR ir_comp){
    uint8_t data = read8_register(ID);
    if(data != 0x7D){
        printf("ISL29125: Error al iniciar. ID esperado 0x7D, obtenido: %02X\n", data);
        return false;
    }
    if(!reset())return false;
    set_mode(mode);
    set_range(range);
    set_resolution(res);
    set_infrared_comp(ir_offset, ir_comp);
    printf("ISL29125: Inicializado correctamente\n");
    return true;
}

void ISL29125::set_mode(RGB_OP_MODES mode){
    uint8_t old_data = read8_register(CONF_1);  // Leemos el byte de conf 1
    old_data &= MASK_MODE;                     // Reseteamos lo que haya
    uint8_t new_data = old_data | uint8_t(mode) ;
    write_register(CONF_1, new_data);
}

void ISL29125::set_range(SENS_RANGE range){
    uint8_t old_data = read8_register(CONF_1);  // Leemos el byte de conf 1
    old_data &= MASK_RANGE;                     // Reseteamos lo que haya
    uint8_t new_data = old_data | uint8_t(range) << 3;
    write_register(CONF_1, new_data);
}

void ISL29125::set_resolution(RES res){
    uint8_t old_data = read8_register(CONF_1);  // Leemos el byte de conf 1
    old_data &= MASK_RESOL;                     // Reseteamos lo que haya
    uint8_t new_data = old_data | uint8_t(res) << 4;
    write_register(CONF_1, new_data);
}

void ISL29125::set_infrared_comp(bool offset, IR comp){
    uint8_t data;
    switch (comp)
    {
    case IR_NONE:
        // data/reg0x02 = [offset? | reserved | 000000]
        data = offset << 7;
        break;
    case IR_LOW:
        // data/reg0x02 = [offset? | reserved | 000011]
        data = offset << 7 | 3;
        break;
    case IR_MEDIUM:
        // data/reg0x02 = [offset? | reserved | 001100]
        data = offset << 7 | 3 << 2;
        break;
    case IR_HIGH:
        // data/reg0x02 = [offset? | reserved | 110000]
        data = offset << 7 | 3 << 4;
        break;
    case IR_MAX:
        // data/reg0x02 = [offset? | reserved | 111111]
        data = offset << 7 | 63;
        break;
    }
    write_register(CONF_2, data);
}

ISL29125::ISL29125Data ISL29125::read_data(){
    read_blue();
    read_green();
    read_red();
    return this->last_measurement;
}

uint16_t ISL29125::read_green(){
    this->last_measurement.green = read16_register(GREEN_LOW);
    return this->last_measurement.green;   
}

uint16_t ISL29125::read_blue(){
    this->last_measurement.blue = read16_register(BLUE_LOW);
    return this->last_measurement.blue;
}

uint16_t ISL29125::read_red(){
    this->last_measurement.red = read16_register(RED_LOW);
    return this->last_measurement.red;
}

uint8_t ISL29125::read8_register(uint8_t reg)
{
    uint8_t cmd[1] = {reg};
    uint8_t read_buf[1];

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    
    if(i2c_write_blocking(i2c_port_, ADDR, cmd, 1, true) != 1){
        mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
        printf("ISL29125: Error al escribir en el registro %02X a 8 bits\n", reg);
        return 0; // Error al tratar de leer
    }
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    if(i2c_read_blocking(i2c_port_, ADDR, read_buf, 1, false) != 1){
        //mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
        printf("ISL29125: Error al leer del registro %02X a 8 bits\n", reg);
        return 0; // Error al tratar de leer
    }
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C

    return read_buf[0];
}

uint16_t ISL29125::read16_register(uint8_t reg){
    uint8_t cmd[1] = {reg};
    uint8_t read_buf[2];

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    
    if(i2c_write_blocking(i2c_port_, ADDR, cmd, 1, true) != 1){
        mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
        printf("ISL29125: Error al escribir en el registro %02X a 16 bits\n", reg);
        return 0; // Error al tratar de escribir
    }
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C

    mutex_enter_blocking(i2c_mutex_); // Asegurar acceso exclusivo al bus I2C
    if(i2c_read_blocking(i2c_port_, ADDR, read_buf, 2, false) != 2){
        mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C
        printf("ISL29125: Error al leer del registro %02X a 16 bits\n", reg);
        return 0; // Error al tratar de leer
    }
    mutex_exit(i2c_mutex_); // Liberar el mutex para permitir otras operaciones I2C

    return (read_buf[1] << 8 | read_buf[0]);
}

void ISL29125::write_register(uint8_t reg, uint8_t val){
    uint8_t buff[2];
    buff[0] = reg;
    buff[1] = val;
    if(i2c_write_blocking(i2c_port_, ADDR, buff, 2, false) != 2) {
        printf("ISL29125: Error al escribir en el registro %02X\n", reg);
        return; // Error al tratar de escribir
    }
}

bool ISL29125::reset(){
    write_register(ID, 0x46);
    uint8_t data = 0x00;
    data = read8_register(CONF_1);
    data |= read8_register(CONF_2);
    data |= read8_register(CONF_3);
    data |= read8_register(STATUS);
    if (data != 0x00){
        printf("ISL29125: No se pudo reiniciar el dispositivo\n");
        return false;
    }
    return true;
}