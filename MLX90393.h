#pragma once
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/mutex.h"
#include <math.h>
#include <stdio.h>

class MLX90393{
public:

    enum RES {
        RESOLUTION_MAX    = 0,      // +- 2^15 (Complemento a dos)
        RESOLUTION_HIGH = 1,        // +- 2^15 (Complemento a dos)
        RESOLUTION_MEDIUM   = 2,    // +- 22000 (Complemento a dos)
        RESOLUTION_LOW    = 3       // +- 11000 (Complemento a dos)
    };

    enum OSR{
        OSR_LOW = 0,
        OSR_MEDIUM = 1,
        OSR_HIGH = 2,
        OSR_MAX = 3
    };
    
    enum DIG_FILT{
        FILT_0 = 0,
        FILT_1 = 1,
        FILT_2 = 2,
        FILT_3 = 3,
        FILT_4 = 4,
        FILT_5 = 5,
        FILT_6 = 6,
        FILT_7 = 7,
    };

    enum GAIN{
        GAIN_0 = 0,
        GAIN_1 = 1,
        GAIN_2 = 2,
        GAIN_3 = 3,
        GAIN_4 = 4,
        GAIN_5 = 5,
        GAIN_6 = 6,
        GAIN_7 = 7,
    };

    typedef struct {
        float x;
        float y;
        float z;
        float t;
        int64_t time_ms;
    } MLX90393Data;

    /**
     * @brief Constructor de la clase, especifica el canal de I2C con el que se trabajara
     */
    MLX90393(i2c_inst_t* i2c_port = i2c0, uint sda_pin = 16, uint scl_pin = 17, uint i2c_freq = 400000, mutex_t* i2c_mutex = nullptr);

    /**
     * @brief Inicializador del sensor
     * @param offset_x offset programable para calibracion del sensor
     * @param offset_y offset programable para calibracion del sensor
     * @param offset_z offset programable para calibracion del sensor
     * @param res_x Resolucion del eje
     * @param res_y Resolucion del eje
     * @param res_z Resolucion del eje
     * @param filt nivel del filtro digital para las mediciones
     * @param osr Oversampling Ratio del CAD para el magnetometro
     * @param osr2 Oversampling Ratio del CAD para el sensor de temperatura
     * @param gain ganancia que se tiene para las mediciones
     */
    bool init_sensor(uint16_t offset_x = 0, 
                        uint16_t offset_y = 0, 
                        uint16_t offset_z = 0, 
                        RES res_x = RESOLUTION_MAX, 
                        RES res_y = RESOLUTION_MAX, 
                        RES res_z = RESOLUTION_MAX, 
                        DIG_FILT filt = FILT_0, 
                        OSR osr = OSR_MEDIUM, 
                        OSR osr2 = OSR_MEDIUM,
                        GAIN gain = GAIN_7);

    /**
     * @brief Metodo que inicia la medicion del magnetometro
     * @param begin_time Tiempo en el que se incia la medicion
     * @return Estructura con los datos para x, y y z
     */
    bool begin_measurement_mag();

    /**
     * @brief Metodo que recaba la medicion del magnetometro
     * @return Estructura con los datos para x, y y z
     */
    MLX90393Data read_measurement_mag();

    /**
     * @brief Metodo que inicia la medicion del sensor de temperatura
     * @param begin_time Tiempo en el que se incia la medicion
     * @return Estructura con los datos para t
     */
    bool begin_measurement_temp();

    /**
     * @brief Metodo que recaba la medicion del sensor de temperatura
     * @return Estructura con los datos para t
     */
    MLX90393Data read_measurement_temp();

    /**
     * @brief Metodo que inicia la medicion del magnetometro y del sensor de temperatura
     * @param begin_time Tiempo en el que se incia la medicion
     * @return Estructura con los datos para x, y, z y t
     */
    bool begin_measurement_mt();

    /**
     * @brief Metodo que recaba la medicion del magnetometro y del sensor de temperatura
     * @return Estructura con los datos para x, y, z y t
     */
    MLX90393Data read_measurement_mt();

    /**
     * @brief Metodo que verifica si se tiene conexion con el sensor
     * @return True si se detecta el sensor, False sino
     */
    bool verify_connection();

    // Tiempo de adquision aproximado para recabar las
    // Mediciones despues de solicitarla al sensor
    // Necesarias por la ausencia de pin de interrupcion
    uint16_t aquisition_time_mag = 0.0f;
    uint16_t aquisition_time_temp = 0.0f;

    MLX90393Data last_measurement;      // Variable que ayuda a guardar los datos de la ultima medicion



private:
    // Direccion I2C
    static constexpr uint8_t ADDR = 0x0C;
    // Comandos
    static constexpr uint8_t SB_XYZ = 0x1E; // Iniciar modo burst en XYZ, no T
    static constexpr uint8_t SM_XYZ = 0x3E; // Iniciar modo Single-measurement en XYZ, no T
    static constexpr uint8_t RM_XYZ = 0x4E; // Extraer lectura en XYZ, no T
    static constexpr uint8_t SM_T = 0x31; // Iniciar modo Single-measurement en T
    static constexpr uint8_t RM_T = 0x41; // Extraer lectura en T
    static constexpr uint8_t SM_XYZT = 0x3F; // Iniciar modo Single-measurement en XYZT
    static constexpr uint8_t RM_XYZT = 0x4F; // Extraer lectura en XYZT
    // Este comando debe completarse con el envio del registro a leer
    static constexpr uint8_t RR = 0x50;     // Leer registro
    // Este comando debe completarse con el envio del dato a escribir (2 bytes) y el registro a escribir
    static constexpr uint8_t WR = 0x60;     // Escribir registro

    i2c_inst_t* i2c_port_;
    uint sda_pin_;
    uint scl_pin_;
    uint i2c_freq_;
    mutex_t* i2c_mutex_;
    float base_sens_x_, base_sens_y_, base_sens_z_;
    RES res_x_, res_y_, res_z_;
    float gain_factor_;
    
    // Multiplicadores de ganancia (Ver la tabla de ganancia y resolucion en el datasheet)
    const float gain_multipliers[8] = {
        5.0f,
        4.0f,
        3.0f,
        2.5f,
        2.0f,
        1.66666667f,
        1.33333333f,
        1.0f
    };

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
     * @brief Calcula el tiempo de adquision para el modo de medicion unica del magnetometro (+30% de olgura)
     * @param filt DIG_FIL asignado por el usuario
     * @param osr OSR asignado por el usuario
     * @return uint16_t Tiempo en microsegundos que se tarda en hacer una medicion
     * @example Para un DIG_FILT y OSR de 0, la medicion tardaria 1357 us
     */
    uint16_t calculate_aquisition_time_mag_us(DIG_FILT filt, OSR osr);

    /**
     * @brief Calcula el tiempo de adquision para el modo de medicion unica del termometro  (+30% de olgura)
     * @param osr2 OSR asignado por el usuario
     * @return uint16_t Tiempo en microsegundos que se tarda en hacer una medicion
     */
    uint16_t calculate_aquisition_time_temp_us(OSR osr2);

};