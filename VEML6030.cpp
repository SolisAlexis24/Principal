#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "VEML6030.h"


VEML6030::VEML6030(i2c_inst_t* i2c_port, uint sda_pin, uint scl_pin, uint i2c_freq, mutex_t* i2c_mutex)
    : i2c_port_(i2c_port), sda_pin_(sda_pin), scl_pin_(scl_pin), i2c_freq_(i2c_freq), i2c_mutex_(i2c_mutex)
    {}

bool VEML6030::init_sensor(ADDR_CONF addr, GAIN gain, INTEGRATION_TIME it){
    ADDR = addr; // Configurar la dirección base del sensor
    uint8_t expected_id;

    switch (ADDR){
    case HIGH: expected_id = 0xD4; break; // ID esperado cuando ADDR está conectado a VCC
    case LOW: expected_id = 0xC4; break; // ID esperado cuando ADDR está conectado a GND
    }
    uint16_t reg07 = read_register(REG_ID); // Leer el registro de identificación para verificar la conexión

    uint8_t saoc = reg07 & 0xFF; // Extraer el SAOC del sensor
    uint8_t id = (reg07 >> 8) & 0xFF; // Extraer el ID del sensor 

    if(id != expected_id) {
        printf("VEML6030: Error de conexión, ID esperado: 0x%02X, ID recibido: 0x%02X\n", expected_id, id);
        return false; // Error al verificar el ID del sensor
    }
    if(saoc != 0x81) {
        printf("VEML6030: Error de conexión, Slave Address Option Code recibido: 0x%02X, esperado: 0x81\n", saoc);
        return false; // Error al verificar el SAOC del sensor
    }

    power_on(); // Encender el sensor
    set_gain(gain); // Configurar la ganancia
    set_integration_time(it); // Configurar el tiempo de integración
    calc_res(); // Calcular la resolución en lux por cuenta
    sleep_ms(10); // Esperar un momento para que el sensor se estabilice
    // enable_PSM(); // Habilitar el modo de ahorro de energía
    // set_power_saving_mode(MODE_1); // Configurar el modo de ahorro de energía
    //printf("VEML6030: Resolución calculada: %.4f lux por cuenta\n", res_);
    //printf("VEML6030: Sensor inicializado correctamente, ID: 0x%02X, SAOC: 0x%02X\n", id, saoc);
    printf("VEML6030: Sensor inicializado correctamente\n");

    return true;
}

void VEML6030::set_gain(GAIN gain){
    uint16_t reg = read_register(REG_CONF); // Leer el registro de configuración actual
    reg &= MASK_GAIN;                          // Limpiar los bits de ganancia (11 y 12)
    reg |= (gain << 11);                    // Establecer la nueva ganancia
    write_register(REG_CONF, reg);          // Escribir el nuevo valor en el registro de configuración
    gain_ = gain;                           // Guardar la ganancia actual
}

void VEML6030::set_integration_time(INTEGRATION_TIME it){
    uint16_t reg = read_register(REG_CONF); // Leer el registro de configuración actual
    reg &= MASK_IT;                         // Limpiar los bits de tiempo de integración (9:6)
    reg |= (it << 6);                       // Establecer el nuevo tiempo de integración
    write_register(REG_CONF, reg);          // Escribir el nuevo valor en el registro de configuración
    it_ = it;                               // Guardar el tiempo de integración actual
}

void VEML6030::set_power_saving_mode(PSM mode){
    uint16_t reg = read_register(POWER_SM); // Leer el registro de modo de ahorro de energía
    reg &= MASK_PSM;                            // Limpiar los bits de modo de ahorro de energía (1:0)
    reg |= (mode << 1);                     // Establecer el nuevo modo de ahorro de energía
    write_register(POWER_SM, reg);          // Escribir el nuevo valor en el registro de modo de ahorro de energía
}

void VEML6030::power_on(){
    uint16_t reg = read_register(REG_CONF); // Leer el registro de configuración actual
    reg &= MASK_POWER;                          // Limpiar el bit de encendido (bit 0)
    reg |= 0x00;                             // Establecer el bit de encendido (bit 0)
    write_register(REG_CONF, reg);          // Escribir el nuevo valor en el registro de configuración
}

void VEML6030::power_off() {
    uint16_t reg = read_register(REG_CONF); // Leer el registro de configuración actual
    reg &= MASK_POWER;                          // Limpiar el bit de encendido (bit 0)
    reg |= 0x01;                             // Establecer el bit de encendido (bit 0)
    write_register(REG_CONF, reg);          // Escribir el nuevo valor en el registro de configuración
}

void VEML6030::enable_PSM(){
    uint16_t reg = read_register(POWER_SM); // Leer el registro de modo de ahorro de energía
    reg &= MASK_EN_PSM;                            // Limpiar el bit de habilitacion de ahorro de energia (1:0)
    reg |= 0x01;                            // Establecer el bit de habilitación del modo de ahorro de energía
    write_register(POWER_SM, reg);          // Escribir el nuevo valor en el registro de modo de ahorro de energía
}

void VEML6030::disable_PSM(){
    uint16_t reg = read_register(POWER_SM); // Leer el registro de modo de ahorro de energía
    reg &= MASK_EN_PSM;                            // Limpiar el bit de habilitacion de ahorro de energia (1:0)
    reg |= 0x00;                            // Deshabilitar el modo de ahorro de energía
    write_register(POWER_SM, reg);          // Escribir el nuevo valor en el registro de modo de ahorro de energía
}

VEML6030::VEML6030Data VEML6030::read_ambient()
{
    uint16_t light_bits = read_register(REG_ALS); // Leer el registro de luz ambiental
    uint32_t lux = calc_lux(light_bits); // Calcular el valor de luz en lux
    this->last_measurement.ambient = lux;
    if (lux > 1000) {
        uint32_t compensated_lux = calc_lux_compensation(lux); // Aplicar compensación si el valor es mayor a 1000 lux
        this->last_measurement.ambient = compensated_lux;
    }
    return this->last_measurement; // Retornar el valor de luz en lux, el cual puede estar compensado
}

VEML6030::VEML6030Data VEML6030::read_white()
{
    uint16_t light_bits = read_register(REG_WHITE); // Leer el registro de luz blanca
    uint32_t lux = calc_lux(light_bits); // Calcular el valor de luz en lux
    this->last_measurement.white = lux;
    if (lux > 1000) {
        uint32_t compensated_lux = calc_lux_compensation(lux); // Aplicar compensación si el valor es mayor a 1000 lux
        this->last_measurement.white = compensated_lux;
    }
    return this->last_measurement; // Retornar el valor de luz en lux, el cual puede estar compensado
}

uint32_t VEML6030::calc_lux(uint16_t light_bits) {
    uint32_t calc_lux = light_bits * res_; // Calcular el valor de luz en lux por cuenta
    return calc_lux;
}

uint32_t VEML6030::calc_lux_compensation(uint32_t lux) {
  uint32_t compLux = (.00000000000060135 * (pow(lux, 4))) -
                      (.0000000093924 * (pow(lux, 3))) +
                      (.000081488 * (pow(lux, 2))) +
                      (1.0023 * lux);
  return compLux;
}

void VEML6030::calc_res(){
    float mult_it_ = 1.f; // Multiplicador para el tiempo de integración
    float mult_gain_ = 1.f; // Multiplicador para la ganancia
    switch (it_)
    {
    case IT_25MS: mult_it_ = 32.f; break;
    case IT_50MS: mult_it_ = 16.f; break;
    case IT_100MS: mult_it_ = 8.f; break;
    case IT_200MS: mult_it_ = 4.f; break;
    case IT_400MS: mult_it_ = 2.f; break;
    case IT_800MS: mult_it_ = 1.f; break;
    }
    switch (gain_)
    {
    case GAIN_1: mult_gain_ = 2.f; break;
    case GAIN_2: mult_gain_ = 1.f; break;
    case GAIN_0_125: mult_gain_ = 16.f; break;
    case GAIN_0_25: mult_gain_ = 8.f; break;
    }
    res_ = base_res_ * mult_it_ * mult_gain_; // Calcula el multiplicador total
}



uint16_t VEML6030::read_register(uint8_t reg) {
    uint8_t buff[2];

    mutex_enter_blocking(i2c_mutex_);
    if(i2c_write_blocking(i2c_port_, ADDR, &reg, 1, true)!=1) {
        printf("VEML6030: Error al escribir en el registro %02X\n", reg);
        mutex_exit(i2c_mutex_);
        return 0; // Error al tratar de escribir
    }
    mutex_exit(i2c_mutex_);
    mutex_enter_blocking(i2c_mutex_);
    if(i2c_read_blocking(i2c_port_, ADDR, buff, 2, false)!=2) {
        printf("VEML6030: Error al leer del registro %02X\n", reg);
        mutex_exit(i2c_mutex_);
        return 0; // Error al tratar de leer
    }
    mutex_exit(i2c_mutex_);
    return ((buff[1] << 8) | buff[0]); // Combina los dos bytes leídos en un uint16_t
}

void VEML6030::write_register(uint8_t reg, uint16_t val) {
    uint8_t buff[3];
    buff[0] = reg;
    buff[1] = val & 0xFF; // Byte bajo
    buff[2] = (val >> 8) & 0xFF; // Byte alto

    if(i2c_write_blocking(i2c_port_, ADDR, buff, 3, false) != 3) {
        printf("VEML6030: Error al escribir en el registro %02X\n", reg);
        return; // Error al tratar de escribir
    }
}
