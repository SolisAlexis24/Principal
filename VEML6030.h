#pragma once
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include <math.h>
#include <stdio.h>

class VEML6030 {
public:
    enum ADDR_CONF {
        HIGH = 0x48,        // Direccion base cuando el pin ADDR esta conectado a VCC
        LOW = 0x10          // Direccion base cuando el pin ADDR esta conectado a GND
    };

    enum INTEGRATION_TIME {
        IT_25MS = 0x0C,     // Integración de 25 ms
        IT_50MS = 0x08,     // Integración de 50 ms
        IT_100MS = 0x00,    // Integración de 100 ms
        IT_200MS = 0x01,    // Integración de 200 ms
        IT_400MS = 0x02,    // Integración de 400 ms
        IT_800MS = 0x03     // Integración de 800 ms
    };

    enum GAIN{
        GAIN_1 = 0x00,      // Ganancia de 1
        GAIN_2 = 0x01,      // Ganancia de 2
        GAIN_0_125 = 0x02,      // Ganancia de 1/8
        GAIN_0_25 = 0x03       // Ganancia de 1/4
    };

    enum PSM{
        MODE_1 = 0x00,     // Modo de potencia 1 (bajo consumo)
        MODE_2 = 0x01,     // Modo de potencia 2 (medio consumo)
        MODE_3 = 0x02,     // Modo de potencia 3 (alto consumo)
        MODE_4 = 0x03      // Modo de potencia 4 (máximo consumo)
    };

    enum MASKS{
        MASK_GAIN = 0xE7FF, // Máscara para la ganancia
        MASK_IT = 0xFC3F,   // Máscara para el tiempo de integración
        MASK_PSM = 0x01,   // Máscara para el modo de ahorro de energía
        MASK_POWER = 0xFFFE, // Máscara para el bit de encendido del sensor
        MASK_EN_PSM = 0x06 // Máscara para el bit de habilitación del modo de ahorro de energía
    };

    typedef struct{
        uint32_t ambient;        // Valor de luz en lux
        uint32_t white;      // Valor de luz blanca
        int64_t time_ms;    // Tiempo de la lectura
    } VEML6030Data;

    /**
     * @brief Constructor de la clase VEML6030
     * 
     * @param i2c_port Puerto I2C a utilizar (i2c0, i2c1, etc.)
     * @param sda_pin Pin SDA del puerto I2C
     * @param scl_pin Pin SCL del puerto I2C
     * @param i2c_freq Frecuencia del bus I2C (en Hz)
     * @param i2c_mutex Mutex para sincronización de acceso al bus I2C
     */
    VEML6030(i2c_inst_t* i2c_port, uint sda_pin, uint scl_pin, uint i2c_freq, mutex_t* i2c_mutex);

    /**
     * @brief Inicializa el sensor VEML6030
     * 
     * @return true Si la inicialización fue exitosa
     * @return false Si hubo un error en la inicialización
     */
    bool init_sensor(ADDR_CONF addr = HIGH, GAIN gain = GAIN_1, INTEGRATION_TIME it = IT_100MS);

    /**
     * @brief Configura la ganancia
     * @param it Ganancia a configurar
     */
    void set_gain(GAIN gain);

    /**
     * @brief Configura el tiempo de integracion
     * @param it Tiempo de integracion a configurar
     */
    void set_integration_time(INTEGRATION_TIME it);

    /**
     * @brief Configura el modo de ahorro de energia
     * @note Si el PSM esta inactivo, esta configuracion no tiene efecto
     */
    void set_power_saving_mode(PSM mode);

    /**
     * @brief Activa completamente el sensor
     */
    void power_on();

    /**
     * @brief Pone al sensor en modo de bajo consumo
     */
    void power_off();

    /**
     * @brief Habilita el modo de ahorro de energía del sensor
     */
    void enable_PSM();

    /**
     * @brief Deshabilita el modo de ahorro de energía del sensor
     */
    void disable_PSM();

    /**
     * @brief Lee el valor de luz ambiental del sensor
     * 
     * @return float Valor de luz ambiental en lux
     */
    VEML6030Data read_ambient();

    /**
     * @brief Lee el valor de luz blanca del sensor
     * 
     * @return float Valor de luz blanca en lux
     */
    VEML6030Data read_white();

    VEML6030Data last_measurement;



    
private:
    uint8_t ADDR = 0x00; // Dirección I2C del sensor

    static constexpr uint8_t REG_CONF = 0x00; // Registro de configuración
    static constexpr uint8_t POWER_SM = 0x03; // Registro de configuración de luz ambiental
    static constexpr uint8_t REG_ALS = 0x04; // Registro de datos de luz ambiental
    static constexpr uint8_t REG_WHITE = 0x05; // Registro de datos de luz blanca
    static constexpr uint8_t REG_ID = 0x07; // Registro de identificación
    static constexpr float base_res_ = 0.0042f; // Resolución base del sensor en lux por cuenta
    
    i2c_inst_t* i2c_port_;
    uint sda_pin_;
    uint scl_pin_;
    uint i2c_freq_;
    mutex_t* i2c_mutex_;
    float res_ = 1.f;   // Resolucion calculada en lux por cuenta, se basa en la ganancia y el tiempo de integración
                        // Ver tabla del datasheet relacionada 
    GAIN gain_ = GAIN_1;  // Ganancia
    INTEGRATION_TIME it_ = IT_100MS;     // Tiempo de integración


    /**
     * @brief Lee un registro del sensor
     * 
     * @param reg Registro a leer
     * @return uint16_t Valor leído del registro
     */
    uint16_t read_register(uint8_t reg);

    /**
     * @brief Escribe en un registro del sensor
     * @param reg Registro a escribir
     * @param val Valor a escribir
     */
    void write_register(uint8_t reg, uint16_t val);

    /**
     * @brief Calcula la resolucion (factor de conversion) basado en la ganancia
     * y el tiempo de integracion
     */
    void calc_res();

    /**
     * @brief Convierte los valores crudos a mediciones en lux/cta
     * @param light_bits Lectura cruda directa del sensor
     * @return Valor retornado
     */
    uint32_t calc_lux(uint16_t light_bits);

    /**
     * @brief Si la lectura es mayor a 1000, indica un comportamiento no lineal,
     * por lo que este debe ser corregido de acuerdo a una funcion proporcionada 
     * por el propio fabricante
     * @param lux Lectura hecha en lux/cta
     * @return Lectura corregida
     */
    uint32_t calc_lux_compensation(uint32_t lux);
};