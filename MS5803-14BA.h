#pragma once

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/mutex.h"
#include "math.h"

/**
 * La manera de usar esta biblioteca es:
 * 1. Solicitar la temperatura
 * 2. Esperar el tiempo de conversion de acuerdo a la presicion seleccionada
 * 3. Tomar la medicion de la temperatura
 * 4. Seolicitar la presion
 * 5. Esperar el tiempo de conversion de acuerdo a la presicion seleccionada
 * 6. Tomar la medicion de la presion
 * El hacerlo de otra forma llevara a errores inevitablemente
 */

class MS5803{
public:
    // Definicion de Oversampling rates, modifican la resolucion de las mediciones y el tiempo de adquisicion
    enum PressureOSR{    // Resolucion media [mbar]
        P_OSR_256 = 0x40,   // 1.0 
        P_OSR_512 = 0x42,   // 0.6
        P_OSR_1024 = 0X44,  // 0.4
        P_OSR_2048 = 0x46,  // 0.3
        P_OSR_4096 = 0x48   // 0.2
    };

    enum TemperatureOSR{     // Resolucion media [C]
        T_OSR_256 = 0x50,   // 0.012
        T_OSR_512 = 0x52,   // 0.008
        T_OSR_1024 = 0X54,  // 0.005
        T_OSR_2048 = 0x56,  // 0.003
        T_OSR_4096 = 0x58   // 0.002
    };


    enum Address
    {
        ADDRESS_HIGH = 0x76,        // Direccion para cuando CS es High
        ADDRESS_LOW = 0x77          // Direccion para cuando CS es Low
    };

    typedef struct{
        float temperature;
        float pressure;
        uint64_t time_ms;
    }MS5803Data;

    /**
     * @brief Constructor que inicializa el sensor con parámetros por defecto
     * 
     * @param i2c_port Puerto I2C a utilizar (por defecto i2c0)
     * @param sda_pin Pin SDA (por defecto GPIO 16)
     * @param scl_pin Pin SCL (por defecto GPIO 17)
     * @param i2c_freq Frecuencia I2C (por defecto 400 kHz)
     */
    MS5803(i2c_inst_t* i2c_port, uint sda_pin, uint scl_pin, uint i2c_freq, mutex_t* i2c_mutex = nullptr);

    /**
     * @brief Metodo que inicializa el sensor. Resetea su memoria y lee los valores del
     * fabricante para correciones
     * @param t_osr Presicion por defecto de la medicion de temperatura
     * @param p_osr Presicion por defecto de la medicion de la presion
     * @param add Direccion I2C del sensor. Puede ser LOW o HIGH dependiendo en el valor del
     * pin CS del sensor. CS = HIGH -> ADD_HIGH, CS = LOW -> ADD_LOW
     */
    bool init_sensor(TemperatureOSR t_osr = T_OSR_4096,
         PressureOSR p_osr = P_OSR_4096, Address add = Address::ADDRESS_HIGH);

    /**
     * @brief Metodo para iniciar la medicion de temperatura con
     * precision por defecto
     * @return True si logro iniciar la medicion. False sino
     */
    bool start_measurement_temp();

    /**
     * @brief Metodo para iniciar la medicion de temperatura con
     * precicion proporcionada como argumento
     * @param t_osr Precision con la que se desea tomar medicion
     * @return True si logro iniciar la medicion. False sino
     */
    bool start_measurement_temp(TemperatureOSR t_osr);

    /**
     * @brief Metodo para convertir la medicion solicitada a [C].
     * Fuertemente basada en lo expresado en el Datasheet.
     * @return Estructura de datos que contiene la ultima temperatura
     * solicitada. Esta tambien se guarda en la variable last_measurement
     * de la instancia del objeto.
     * @attention Si se lee la presion de la variable last_measurement
     * despues de usar esta funcion y obtener un resultado exitoso, lo que
     * se reflejara sera, un valor 0 si es la primera vez que se ejecuta o
     * el ultimo valor de presion que se calculo en la anterior medicion.
     */
    MS5803Data read_measurement_temp();

    /**
     * @brief Metodo para iniciar la medicion de presion con
     * precision por defecto,
     * @return True si logro iniciar la medicion. False sino.
     */
    bool start_measurement_press();

    /**
     * @brief Metodo para iniciar la medicion de presion con
     * precicion proporcionada como argumento,
     * @param p_osr Precision con la que se desea tomar medicion.
     * @return True si logro iniciar la medicion. False sino.
     */
    bool start_measurement_press(PressureOSR p_osr);

    /**
     * @brief Metodo para convertir la medicion solicitada a [mbar].
     * Fuertemente basada en lo expresado en el Datasheet.
     * @return Estructura de datos que contiene la ultima presion
     * solicitada. Esta tambien se guarda en la variable last_measurement
     * de la instancia del objeto.
     * @attention Si se lee la temperatura de la variable last_measurement
     * despues de usar esta funcion y obtener un resultado exitoso, lo que
     * se reflejara sera la ultima temperatura registrada.
     */
    MS5803Data read_measurement_press();


    MS5803Data last_measurement;        // Inmformacion de la ultima medicion realizada

    uint8_t acquisition_time;            // Tiempo de adquisicion maximo para la medicion



private:

    TemperatureOSR default_temp_osr;
    PressureOSR    default_press_osr;
    // Informacion de calibracion de fabrica
    uint16_t C1;            // Sensivibilad de presion
    uint16_t C2;            // Offset de presion
    uint16_t C3;            // Coeficiente termico de sensibilidad de presion
    uint16_t C4;            // Coeficiente termico de offset de presion
    uint16_t C5;            //Temperatura de referencia
    uint16_t C6;            // Coeficiente termico de temperatura
    uint8_t CRC;            // CRC de los valores de la EPROM

    // Valores raw leidos
    int32_t D1;            // Valor digital de presion
    int32_t D2;            // Valor digital de temperatura

    // Calculo de la presion
    int32_t dT;             // Diferencia entre temperatura actual y referencia
    int32_t TEMP;           // Temperatura actual

    // Calculo de la presion
    int64_t OFF;            // Offset de la temperatura actual
    int64_t SENS;           // Sensibilidad a la temperatura actual
    int32_t P;              // Presion comprensada con la temperatura

    // Para compensacion de segundo orden
    int64_t T2;             // Temperatura actual 2
    int64_t OFF2;           // Offset de la temperatura actual  2
    int64_t SENS2;           // Sensibilidad a la temperatura actual 2   

    uint8_t ADDR = 0x00;  // Direccion del sensor
    // Registros de la EPROM
    static constexpr uint8_t C1_REG = 0xA2;
    static constexpr uint8_t C2_REG = 0xA4;
    static constexpr uint8_t C3_REG = 0xA6;
    static constexpr uint8_t C4_REG = 0xA8;
    static constexpr uint8_t C5_REG = 0xAA;
    static constexpr uint8_t C6_REG = 0xAC;
    static constexpr uint8_t CRC_REG = 0xAE;
    // Comandos
    static constexpr uint8_t R_ADC = 0x00;
    static constexpr uint8_t RESET = 0x1E;

    

    i2c_inst_t* i2c_port_;
    uint sda_pin_;
    uint scl_pin_;
    uint i2c_freq_;
    mutex_t *i2c_mutex_; // Mutex para asegurar acceso exclusivo al bus I2C

    /**
     * @brief Funcion que sirve para leer los valores del fabricante desde la PROM.
     * @param reg Registro desde el que se leeran los valores.
     * @return Valor del registro si fue exitosa la lectura.
     */
    uint16_t read_from_prom(uint8_t reg);

    /**
     * @brief Metodo para conseguir la ultima medicion solicitada al sensor.
     * @return Ultima medicion realizada por el sensor.
     */
    uint32_t get_measurement();

    /**
     * @brief Metodo que verifica el CRC de los valores leidos de la EPROM.
     * @param prom_coeffs Valores leidos de la EPROM.
     * @param crc_read Valor del CRC leido de la EPROM.
     * @return True si el CRC es correcto, false en caso contrario.
     * @note El CRC se calcula de acuerdo a lo expresado en el Datasheet del sensor.
     */
    bool verify_crc(uint16_t prom_coeffs[8], uint8_t crc_read);

    /**
     * @brief Metodo que resetea el sensor para comenzar a utilizarlo
     */
    bool reset();

    /** 
     * @brief Metodo que calcula el mayor tiempo de adquisicion de entre los dos posibles
     * de acuerdo a las precisiones seleccionadas.
     * @param t_osr Precision de temperatura
     * @param p_osr Precision de presion
    */
    uint8_t calculate_acquisition_time_ms(TemperatureOSR t_osr, PressureOSR p_osr);

};
