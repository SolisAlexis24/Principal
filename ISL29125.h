#pragma once
#include "hardware/i2c.h"
#include "pico/mutex.h"

class ISL29125{
    public:
    // Modos de operacion configurados en el registro 0x00 [B2:B0]
    enum RGB_OP_MODES{ 
        MODE_POWER_DOW = 0,      // Power down
        MODE_GREEN_ONLY = 1,     // Green only
        MODE_RED_ONLY = 2,       // Red only
        MODE_BLUE_ONLY = 3,      // Blue only
        MODE_STD_BY = 4,         // No conversion
        MODE_RGB = 5,            // Green, red & blue 
        MODE_RG = 6,             // Red & green
        MODE_GB = 7              // Green & blue
    };

    // Rangos de sensibilidad configurados en el registro 0x00 [B3]
    // Determina la resolucion de la medicion del ADC
    enum SENS_RANGE{
        RAN_375 = 0,          // [0,375] lux
        RAN_10_000 = 1        // [0, 10000] lux
    };
    // Resoluciones configuradas en el registro 0x00 [B4]
    // Cambiar la resolucion afecta el tiempo de muestreo
    enum RES{
        RES_16 = 0,           // 16 bits de resolucion
        RES_12 = 1            // 12 bits de resolucion
    };

    //Compensacion activa de luz infrarroja configurada en el registro 2
    enum IR{
        IR_NONE = 0,              // No enciende ningun bit
        IR_LOW = 1,              // Enciende los bits B0 y B1 = 3
        IR_MEDIUM = 2,           // Enciende los bits B2 y B3 = 12
        IR_HIGH = 3,             // Enciende los bits B4 y B5 = 48
        IR_MAX = 4               // Enciende todos los bits = 63
    };

    enum MASKS{
        MASK_MODE = 0xF8,          // Lleva a 0 los bits del modo en Reg0x00
        MASK_RANGE = 0xF7,         // Lleva a 0 el bit del rango en Reg0x00
        MASK_RESOL = 0xEF,         // Lleva a 0 el bit de la resolucion en Reg0x00
        MASK_IR_OFFSET = 0x7F,     // Lleva a 0 el bit de offset de compensacion ir
    };

    typedef struct {
        uint16_t green;
        uint16_t red;
        uint16_t blue;
        int64_t time_ms;
    } ISL29125Data;

    /**
     * @brief Constructor de la clase VEML6030
     * 
     * @param i2c_port Puerto I2C a utilizar (i2c0, i2c1, etc.)
     * @param sda_pin Pin SDA del puerto I2C
     * @param scl_pin Pin SCL del puerto I2C
     * @param i2c_freq Frecuencia del bus I2C (en Hz)
     * @param i2c_mutex Mutex para sincronización de acceso al bus I2C
     */
    ISL29125(i2c_inst_t* i2c_port, uint sda_pin, uint scl_pin, uint i2c_freq, mutex_t* i2c_mutex);

    /**
     * @brief Este metono inicaliza el sensor, verificando su ID para determinar si esta correctamente conectado
     * @return true si se inicializo correctamente, false en otro caso
     */
    bool init_sensor(RGB_OP_MODES mode = MODE_RGB, SENS_RANGE range = RAN_10_000, RES res = RES_16, bool ir_offset = false, IR ir_comp = IR_LOW);

    /**
     * @brief Este metodo configura el modo de operacion
     * @param mode modo seleccionado
     */
    void set_mode(RGB_OP_MODES mode);

    /**
     * @brief Este metodo configura el rango seleccionado
     * @param range Rango seleccionada
     */
    void set_range(SENS_RANGE range);

    /**
     * @brief Este metodo configura la resolucion de las mediciones a 16 o 12 bits
     * @param res Resolucion seleccionada
     */
    void set_resolution(RES res);

    /**
     * @brief Este metodo configura la compensacion por luz infraroja
     * @param offset Este pearametro indica si se enciende o no el offset de 106 unidades (B7@Reg0x02)
     * @param comp Tipo de compresacion
     */
    void set_infrared_comp(bool offset, IR comp);

    /**
     * @brief este metodo lee los registros de informacion necesarios para la medicion
     * @return Estructura de datos con la informaicon
     * @note La cantidad de informacion depende del modo de operacion seleccionado por el usuario
     */
    ISL29125Data read_data();

    /**
     * @brief Este metodo lee el contenido del registro de informacion relacionado a la lectura del color verde
     * @return Lectura realizada
     */
    uint16_t read_green();

    /**
     * @brief Este metodo lee el contenido del registro de informacion relacionado a la lectura del color azul
     * @return Lectura realizada
     */
    uint16_t read_blue();

    /**
     * @brief Este metodo lee el contenido del registro de informacion relacionado a la lectura del color rojo
     * @return Lectura realizada
     */
    uint16_t read_red();

    ISL29125Data last_measurement;

    

    private:
    // Direccion del sensor
    static constexpr uint8_t ADDR = 0x44;
    // Registros internos del sensor
    // Se omiten los registros relacionados enteramente a interrupciones
    static constexpr uint8_t ID = 0x00;
    static constexpr uint8_t CONF_1 = 0x01;
    static constexpr uint8_t CONF_2 = 0x02;
    static constexpr uint8_t CONF_3 = 0x03;
    static constexpr uint8_t STATUS = 0x08;
    static constexpr uint8_t GREEN_LOW = 0x09;
    static constexpr uint8_t GREEN_HIGH = 0x0A;
    static constexpr uint8_t RED_LOW = 0x0B;
    static constexpr uint8_t RED_HIGH = 0x0C;
    static constexpr uint8_t BLUE_LOW = 0x0D;
    static constexpr uint8_t BLUE_HIGH = 0x0E;

    i2c_inst_t* i2c_port_;
    uint sda_pin_;
    uint scl_pin_;
    uint i2c_freq_;
    mutex_t* i2c_mutex_;

    /**
     * @brief Lee el contenido de un registro de 8 bits
     * @param reg Registro a leer
     * @return Lectura del registro realizada
     */
    uint8_t read8_register(uint8_t reg);

    /**
     * @brief Lee el contenido de un registro de 16 bits
     * @param reg Registro a leer
     * @return Lectura del registro realizada
     */
    uint16_t read16_register(uint8_t reg);

    /**
     * @brief Escribe en un registro
     * @param reg Registro a ser escrito
     * @param val Valor a ser escrito
     */
    void write_register(uint8_t reg, uint8_t val);

    /**
     * @brief Metodo que reinica los valores de los registros del sensor
     */
    bool reset();

};